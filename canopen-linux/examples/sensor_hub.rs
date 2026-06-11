//! Multi-sensor hub node — demonstrates multiple TPDOs, EMCY, and threshold monitoring.
//!
//! This node simulates a sensor hub with:
//! - 4 temperature sensors (0x6000, sub1-4, i16 in 0.1C units)
//! - 2 analog inputs (0x6010, sub1-2, u16 raw ADC values)
//! - Configurable alarm thresholds (0x6100)
//! - TPDO1: temperatures at 100ms event timer
//! - TPDO2: analog inputs at 250ms event timer
//! - EMCY on over-temperature
//!
//! Usage:
//!   CAN_TRANSPORT=udp cargo run --example sensor_hub
//!
//! ## API observations
//!
//! 1. **Multiple TPDOs with different rates works well**: The per-TPDO event_timer
//!    configuration is flexible. TPDO1 sends temperatures every 100ms, TPDO2
//!    sends analog values every 250ms. This is a clean design.
//!
//! 2. **OD value validation via `#[validate_write(fn)]`**: This example uses
//!    `#[validate_write(check_thresholds)]` to reject `temp_high < temp_low`
//!    and `adc_high < adc_low` before the SDO write commits. The master
//!    receives an SDO abort with `ValueRangeExceeded` instead of silently
//!    getting an inconsistent configuration.
//!
//! 3. **EMCY error codes are fine for standard codes, but awkward for
//!    device-profile-specific codes**: `EmcyErrorCode` has predefined variants,
//!    but device profiles (e.g., CiA 404 for measurement devices) define their
//!    own error codes. Currently you pass a raw u16 to `set_error()`, which
//!    works but bypasses the type system. Either accept both enum and raw, or
//!    document that raw codes are the expected pattern for device profiles.
//!
//! 4. **OdGuard auto-diff is excellent for TPDO triggering**: Writing to
//!    temperature fields via `od_mut()` automatically marks them dirty, which
//!    triggers event-driven TPDOs. No manual `notify_changed()` needed. This
//!    is the best part of the API for sensor-type applications.
//!
//! 5. **No way to dynamically enable/disable PDO mappings**: If a sensor fails,
//!    you might want to remove it from the TPDO mapping. Currently PDO configs
//!    are set at init time. While you can modify them via `tpdo_engine_mut()`,
//!    the API for this is low-level (edit the config struct directly). A
//!    `set_tpdo_enabled(index, bool)` helper would be useful.
//!
//! 6. **Event queue overflow tracking**: The `EVT_QUEUE` const generic defaults
//!    to 16. `Node::events_dropped()` now tracks how many events were lost to
//!    overflow, making it easy to detect if the queue is undersized.

use canopen_core::cobid::NodeId;
use canopen_core::lss::LssIdentity;
use canopen_core::node::{Node, NodeConfig};
use canopen_core::time::Clock;
use canopen_core::transport::CanFrame;
use canopen_derive::object_dictionary;
use canopen_linux::{SocketcanTransport, UdpMulticastTransport};
use std::time::Instant;

object_dictionary! {
    #[validate_write(check_thresholds)]
    pub struct SensorHubOd {
        // Standard objects
        [0x1000] device_type: u32 = 0x0000_0194, ro;  // 0x194 = measurement device
        [0x1001] error_register: u8 = 0x00, ro;
        [0x1017] heartbeat_time: u16 = 500, rw;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0x0000_CAFE, ro;
            [2] product_code: u32 = 0x0003, ro;
            [3] revision: u32 = 0x0001_0000, ro;
            [4] serial_number: u32 = 0x0000_0003, ro;
        };

        // Temperature sensors: i16, units = 0.1 degC (e.g., 251 = 25.1C)
        // All 4 packed into TPDO1 = 8 bytes exactly (4 x i16 = 8 bytes)
        [0x6000] temperatures: record {
            [1] temp1: i16 = 250, ro, pdo;  // 25.0 C
            [2] temp2: i16 = 251, ro, pdo;  // 25.1 C
            [3] temp3: i16 = 249, ro, pdo;  // 24.9 C
            [4] temp4: i16 = 252, ro, pdo;  // 25.2 C
        };

        // Analog inputs: raw 12-bit ADC values (0-4095)
        // TPDO2 = 4 bytes (2 x u16)
        [0x6010] analog: record {
            [1] adc1: u16 = 2048, ro, pdo;
            [2] adc2: u16 = 1024, ro, pdo;
        };

        // Alarm thresholds (configurable via SDO by the master)
        // Validated by check_thresholds() — rejects temp_high < temp_low
        // and adc_high < adc_low via SDO abort before the write commits.
        [0x6100] alarms: record {
            [1] temp_high: i16 = 800, rw, pdo;   // 80.0 C — over-temperature alarm
            [2] temp_low: i16 = -100, rw, pdo;    // -10.0 C — under-temperature alarm
            [3] adc_high: u16 = 3900, rw;    // ~95% of range
            [4] adc_low: u16 = 100, rw;      // ~2.5% of range
        };

        // TPDO1: all 4 temperatures, event-driven with 100ms timer
        // inhibit_time = 100 (10ms minimum between sends, in 100us units)
        tpdo[1](transmission_type = 255, inhibit_time = 100, event_timer = 100) {
            temp1,
            temp2,
            temp3,
            temp4,
        };

        // TPDO2: analog inputs, event-driven with 250ms timer
        tpdo[2](transmission_type = 255, inhibit_time = 100, event_timer = 250) {
            adc1,
            adc2,
        };

        // RPDO1: receive threshold updates from master
        // (alternative to SDO — allows real-time threshold adjustment)
        rpdo[1](transmission_type = 255) {
            temp_high,
            temp_low,
        };
    }
}

impl SensorHubOd {
    /// Validate threshold writes: reject temp_high < temp_low and adc_high < adc_low.
    fn check_thresholds(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
    ) -> Result<(), canopen_core::od::OdError> {
        match (index, subindex) {
            // temp_high being written — check it's >= current temp_low
            (0x6100, 1) if data.len() >= 2 => {
                let new_high = i16::from_le_bytes([data[0], data[1]]);
                if new_high < self.temp_low {
                    return Err(canopen_core::od::OdError::ValueRange);
                }
            }
            // temp_low being written — check it's <= current temp_high
            (0x6100, 2) if data.len() >= 2 => {
                let new_low = i16::from_le_bytes([data[0], data[1]]);
                if new_low > self.temp_high {
                    return Err(canopen_core::od::OdError::ValueRange);
                }
            }
            // adc_high being written — check >= current adc_low
            (0x6100, 3) if data.len() >= 2 => {
                let new_high = u16::from_le_bytes([data[0], data[1]]);
                if new_high < self.adc_low {
                    return Err(canopen_core::od::OdError::ValueRange);
                }
            }
            // adc_low being written — check <= current adc_high
            (0x6100, 4) if data.len() >= 2 => {
                let new_low = u16::from_le_bytes([data[0], data[1]]);
                if new_low > self.adc_high {
                    return Err(canopen_core::od::OdError::ValueRange);
                }
            }
            _ => {}
        }
        Ok(())
    }
}

struct StdClock {
    start: Instant,
}

impl StdClock {
    fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Clock for StdClock {
    fn now_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}

/// Simulate sensor readings with slight drift and occasional spikes.
struct SensorSimulator {
    tick: u64,
}

impl SensorSimulator {
    fn new() -> Self {
        Self { tick: 0 }
    }

    fn update(&mut self) -> SensorReadings {
        self.tick += 1;
        let t = self.tick as f64 * 0.01; // slow time base

        // Temperatures: base 25C with sinusoidal variation + noise
        let base_temp = 250.0; // 25.0 C in 0.1C units
        let readings = SensorReadings {
            temp1: (base_temp + 20.0 * (t * 0.3).sin() + 5.0 * (t * 1.7).sin()) as i16,
            temp2: (base_temp + 15.0 * (t * 0.4).sin() + 3.0 * (t * 2.1).cos()) as i16,
            temp3: (base_temp + 25.0 * (t * 0.2).sin()) as i16,
            // temp4: occasional spike to test alarm
            temp4: if self.tick % 500 == 0 {
                850 // 85.0 C spike — should trigger alarm if threshold is 80.0 C
            } else {
                (base_temp + 10.0 * (t * 0.5).cos()) as i16
            },
            adc1: ((2048.0 + 1000.0 * (t * 0.1).sin()) as u16).clamp(0, 4095),
            adc2: ((1024.0 + 500.0 * (t * 0.15).cos()) as u16).clamp(0, 4095),
        };
        readings
    }
}

struct SensorReadings {
    temp1: i16,
    temp2: i16,
    temp3: i16,
    temp4: i16,
    adc1: u16,
    adc2: u16,
}

fn run_node(transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>) {
    let node_id = NodeId::new(3).unwrap();
    let od = SensorHubOd::new();

    let config = NodeConfig::<2, 1> {
        node_id,
        heartbeat_interval_ms: 500,
        auto_start: false,
        tpdo: od.tpdo_configs(node_id),
        rpdo: od.rpdo_configs(node_id),
        identity: LssIdentity {
            vendor_id: 0xCAFE,
            product_code: 0x0003,
            revision: 0x00010000,
            serial: 0x00000003,
        },
    };

    // Note the const generics: 2 TPDOs, 1 RPDO, 32-event queue (more headroom
    // since we update 6 values per cycle), 8 dirty set entries.
    let mut node: Node<SensorHubOd, 2, 1, 32, 8> = Node::new(config, od);
    let clock = StdClock::new();
    let mut simulator = SensorSimulator::new();

    // Track active alarms to avoid spamming EMCY
    let mut temp_alarm_active = [false; 4];

    eprintln!("Sensor hub node {} running", node_id.raw());
    eprintln!(
        "TPDO1 (temps):  0x{:03X}, 100ms event timer",
        0x180 + node_id.raw() as u16
    );
    eprintln!(
        "TPDO2 (analog): 0x{:03X}, 250ms event timer",
        0x280 + node_id.raw() as u16
    );
    eprintln!("RPDO1 (thresh): 0x{:03X}", 0x200 + node_id.raw() as u16);

    let mut last_print = Instant::now();

    loop {
        node.process(transport, &clock);

        // Update sensor readings every 10ms
        let readings = simulator.update();
        {
            // OdGuard auto-diff: changed values automatically trigger TPDO
            let mut od = node.od_mut();
            od.temp1 = readings.temp1;
            od.temp2 = readings.temp2;
            od.temp3 = readings.temp3;
            od.temp4 = readings.temp4;
            od.adc1 = readings.adc1;
            od.adc2 = readings.adc2;
        }

        // Check alarm thresholds
        let temps = [
            readings.temp1,
            readings.temp2,
            readings.temp3,
            readings.temp4,
        ];
        let high_thresh = node.od().temp_high;
        let low_thresh = node.od().temp_low;

        for (i, &temp) in temps.iter().enumerate() {
            let over = temp > high_thresh || temp < low_thresh;

            if over && !temp_alarm_active[i] {
                // New alarm — send EMCY
                //
                // API OBSERVATION: set_error() takes a raw u16 error code.
                // CiA 404 (measurement devices) defines specific codes like
                // 0x4200 (temperature sensor error). We pass raw codes, which
                // works but isn't type-checked. The EmcyErrorCode enum covers
                // generic codes but not device-profile-specific ones.
                let vendor_data = [(i + 1) as u8, 0, 0, 0, 0]; // sensor index in vendor bytes
                node.set_error(
                    0x4200, // Temperature error (CiA 404)
                    canopen_core::error_register::GENERIC
                        | canopen_core::error_register::TEMPERATURE,
                    &vendor_data,
                );
                eprintln!(
                    "ALARM: Sensor {} temp={:.1}C exceeds threshold",
                    i + 1,
                    temp as f64 / 10.0
                );
                temp_alarm_active[i] = true;
            } else if !over && temp_alarm_active[i] {
                // Alarm cleared
                temp_alarm_active[i] = false;
                if !temp_alarm_active.iter().any(|&a| a) {
                    node.clear_error(canopen_core::error_register::TEMPERATURE);
                    eprintln!("All temperature alarms cleared");
                }
            }
        }

        // Handle OD events (threshold changes from SDO or RPDO)
        while let Some(evt) = node.next_event() {
            match (evt.index, evt.subindex) {
                (0x6100, 1) => {
                    eprintln!(
                        "Threshold updated: temp_high = {:.1}C",
                        node.od().temp_high as f64 / 10.0
                    );
                }
                (0x6100, 2) => {
                    eprintln!(
                        "Threshold updated: temp_low = {:.1}C",
                        node.od().temp_low as f64 / 10.0
                    );
                }
                (0x6100, 3) => eprintln!("Threshold updated: adc_high = {}", node.od().adc_high),
                (0x6100, 4) => eprintln!("Threshold updated: adc_low = {}", node.od().adc_low),
                _ => {}
            }
        }

        // Periodic status print
        if last_print.elapsed() >= std::time::Duration::from_secs(5) {
            let dropped = node.events_dropped();
            eprintln!(
                "temps=[{:.1} {:.1} {:.1} {:.1}]C  adc=[{} {}]  state={:?}  alarms={:?}{}",
                readings.temp1 as f64 / 10.0,
                readings.temp2 as f64 / 10.0,
                readings.temp3 as f64 / 10.0,
                readings.temp4 as f64 / 10.0,
                readings.adc1,
                readings.adc2,
                node.state(),
                temp_alarm_active,
                if dropped > 0 {
                    format!("  WARN: {} events dropped", dropped)
                } else {
                    String::new()
                }
            );
            last_print = Instant::now();
        }

        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn main() {
    let transport_type = std::env::var("CAN_TRANSPORT").unwrap_or("socketcan".into());

    match transport_type.as_str() {
        "udp" => {
            eprintln!("Sensor hub on UDP multicast (239.74.163.2:43113)");
            let mut transport = UdpMulticastTransport::new(None, None)
                .expect("Failed to create UDP multicast transport");
            run_node(&mut transport);
        }
        _ => {
            let iface = std::env::var("CAN_IFACE").unwrap_or("vcan0".into());
            eprintln!("Sensor hub on {}", iface);
            let mut transport =
                SocketcanTransport::open(&iface).expect("Failed to open CAN interface");
            run_node(&mut transport);
        }
    }
}
