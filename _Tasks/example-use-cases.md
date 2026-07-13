# Example Use Cases From Local Consumers

> **Status: survey + example plan (2026-07-06).**
> Based on a scan of `~/src` for local users of `canopen-core`,
> `canopen-linux`, and `canopen-derive`.

## Projects found

Active Rust users:

- `frost/frost-arm-can-bridge`
  - std bridge/control loop for six Nanotec CL4-E CiA 402 drives.
  - Uses SocketCAN/SLCAN, typed SDO client generated from vendor EDS, NMT,
    RPDO command frames, TPDO/heartbeat/EMCY monitoring.
- `frost/frost-cia309-modbus-tcp-gateway`
  - std Modbus/TCP to CANopen gateway.
  - Exposes CiA 309-2 style SDO upload/download and NMT over SocketCAN.
- `frost/frost_ee_test`
  - std egui tool for monitoring and configuring a CANopen endeffector.
  - Background CAN thread forwards raw frames to the UI and serves SDO/NMT
    commands.
- `frost/nanotec_canopen_test`
  - std egui tool for Nanotec drives.
  - Same pattern as `frost_ee_test`: raw frame monitor plus SDO mailbox.
- `frost/frost-arm-ros1-ros2-gateway`
  - std ROS gateway with an endeffector CANopen helper.
  - Uses SDO and frame-level parsing similarly to the test tools.
- `frost/frost-res-ros2-gateway`
  - std ROS2 gateway decoding TPDO/EMCY/heartbeat and sending RPDO control.
  - Uses transports and `CanFrame`, but hand-decodes CANopen frames.
- `h2/msr-polyguard-h2-can-firmware/mcu`
  - Embassy/no_std CANopen sensor node.
  - Publishes sensor data via TPDO, serves OD via SDO, exports EDS.
- `h2/msr-polyguard-h2-can-firmware/simulator`
  - std simulator for the same sensor node.
  - Runs a `Node<OD>` against SocketCAN/SLCAN.
- `h2/display/mcu/mcu`
  - Embassy/no_std display node consuming another node's TPDOs.
  - Local `Node<OD>` receives RPDOs mapped from the H2 sensor TPDO COB-IDs.
- `frost/frost-endeffector-v_pfusch-firmware/mcu`
  - Embassy/no_std actuator node.
  - Rich OD, TPDO/RPDO, persistent config via OD command object, EMCY producer.

## Use case 1: std SDO/NMT gateway

Representative projects:

- `frost-cia309-modbus-tcp-gateway`
- parts of `frost-arm-ros1-ros2-gateway`

Big picture:

A synchronous server protocol receives requests from a client (Modbus/TCP,
ROS service, CLI, etc.) and translates them into CANopen SDO uploads/downloads
or NMT frames. It does not primarily care about continuous PDO processing.

Current pain:

- Each app hand-drives `SdoClient` with timeout loops.
- Stale SDO responses and unrelated frames are handled ad hoc.
- There is no clean std API for "own CAN in a worker, expose SDO/NMT methods".

Example to publish:

`canopen-linux/examples/sdo_gateway.rs`

Sketch:

```rust
fn main() -> anyhow::Result<()> {
    let can = SocketcanTransport::open("can0")?;
    let gateway = CanOpenGateway::spawn(can);

    let node = NodeId::new(0x21).unwrap();

    gateway.nmt_start(node)?;
    let device_type = gateway.sdo(node).read_u32(0x1000, 0)?;
    gateway.sdo(node).write_u16(0x1017, 0, 250)?;

    println!("device type = 0x{device_type:08x}");
    Ok(())
}
```

API pressure:

- std blocking `SdoHandle` with timeout;
- NMT helpers;
- no raw-frame ownership by the SDO transfer;
- clear error mapping for abort/timeout/transport.

## Use case 2: std monitor UI + SDO mailbox

Representative projects:

- `frost_ee_test`
- `nanotec_canopen_test`

Big picture:

A desktop UI wants all raw CAN frames for display/classification, while also
issuing SDO read/write requests from UI actions. SDO must not steal frames
from the monitor; unrelated frames observed during SDO transfers should still
reach the UI.

Current pain:

- The CAN thread forwards non-SDO frames manually during SDO loops.
- The UI does its own `CobId::parse()` classification.
- Bulk SDO reads and live bus monitoring share one transport awkwardly.

Example to publish:

`canopen-linux/examples/can_monitor_with_sdo.rs`

Sketch:

```rust
fn main() -> anyhow::Result<()> {
    let bus = CanOpenBus::spawn(SocketcanTransport::open("can0")?);
    let mut events = bus.subscribe_raw::<128>();
    let sdo = bus.sdo_client(NodeId::new(0x50).unwrap());

    std::thread::spawn(move || {
        while let Ok(frame) = events.recv() {
            let kind = CanOpenEvent::decode(frame);
            println!("{kind:?} {frame:?}");
        }
    });

    let device_type = sdo.read_u32(0x1000, 0)?;
    let name = sdo.upload(0x1008, 0)?;

    println!("device_type=0x{device_type:08x}, name={name:?}");
    Ok(())
}
```

API pressure:

- raw frame subscription;
- decoded CANopen event helper;
- SDO client as independent consumer;
- backpressure/overflow reporting for UI event queues.

## Use case 3: std CiA 402 multi-axis controller

Representative project:

- `frost-arm-can-bridge`

Big picture:

A controller configures several CiA 402 drives by SDO, starts them with NMT,
sends RPDO command frames at control-loop cadence, and consumes TPDO,
heartbeat, and EMCY feedback to maintain a shared process image.

Current pain:

- The app combines SDO sequencing, PDO parsing, heartbeat monitoring, EMCY
  fault state, and raw RPDO construction by hand.
- Typed SDO client generation is valuable, but it needs a better bus model
  underneath.
- The same app needs both high-level typed SDO and low-level PDO frame control.

Example to publish:

`canopen-linux/examples/cia402_velocity_controller.rs`

Sketch:

```rust
canopen_derive::sdo_client_from_eds! {
    pub struct DriveClient = "examples/eds/cl4e.eds";
}

fn main() -> anyhow::Result<()> {
    let bus = CanOpenBus::spawn(SocketcanTransport::open("can0")?);
    let drive = NodeId::new(0x21).unwrap();
    let sdo = DriveClient::new(drive, bus.sdo_port(drive));
    let mut events = bus.subscribe_canopen::<64>();

    sdo.write_i1017_s00_producer_heartbeat_time(125)?;
    sdo.write_i6060_s00_modes_of_operation(3)?;
    sdo.write_i6040_s00_controlword(0x0006)?;
    sdo.write_i6040_s00_controlword(0x0007)?;
    sdo.write_i6040_s00_controlword(0x000f)?;

    loop {
        bus.send(make_rpdo3(drive, target_velocity(), false))?;

        while let Some(event) = events.try_recv() {
            match event {
                CanOpenEvent::Tpdo { node, pdo_num: 1, frame } if node == drive => {
                    update_position_velocity(frame.data());
                }
                CanOpenEvent::Heartbeat { node, state } if node == drive => {
                    update_nmt_state(state);
                }
                CanOpenEvent::Emcy(msg) if msg.node == drive => {
                    fault(msg);
                }
                _ => {}
            }
        }
    }
}
```

API pressure:

- typed client from EDS with address-first names;
- shared bus event stream;
- helpers for CiA 402 state transitions might be a later crate/module;
- easy raw RPDO transmission.

## Use case 4: no_std Embassy sensor node

Representative projects:

- `h2/msr-polyguard-h2-can-firmware/mcu`
- `frost-endeffector-v_pfusch-firmware/mcu`

Big picture:

An MCU is a CANopen node. It owns an object dictionary, publishes TPDOs from
local sensor/control state, accepts SDO/RPDO writes, emits heartbeat and EMCY,
and optionally persists parameters.

Current pain:

- Every firmware repeats the same channel-based CAN transport wrapper.
- `Node::process()` polling is clear but boilerplate-heavy around Embassy CAN
  RX/TX tasks.
- Current OD event queue APIs are transitional and slated for the node
  application-model refactor.

Example to publish:

`examples/embassy-sensor-node`

Sketch:

```rust
object_dictionary! {
    #[export_eds(path = "../sensor.eds")]
    pub struct SensorOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x1001] error_register: u8 = 0, ro;
        [0x6000] measurement: record {
            [1] value: u32 = 0, ro, pdo;
            [2] status: u8 = 0, ro, pdo;
        };

        tpdo[1](transmission_type = 255, event_timer = 100ms) {
            value,
            status,
        };
    }
}

#[embassy_executor::task]
async fn protocol_task() {
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;
    let mut ticker = Ticker::every(Duration::from_millis(1));

    loop {
        ticker.next().await;
        NODE.with(|node| node.process(&mut transport, &clock));
    }
}

#[embassy_executor::task]
async fn sensor_task() {
    loop {
        let sample = read_sensor().await;
        NODE.with(|node| {
            let mut od = node.od_mut();
            od.value = sample.value;
            od.status = sample.status;
        });
        Timer::after_millis(10).await;
    }
}
```

API pressure:

- reusable Embassy CAN RX/TX channel transport pattern;
- model-3 scan/update ergonomics;
- EDS export;
- EMCY producer API.

## Use case 5: no_std mixed node consuming remote TPDOs

Representative project:

- `h2/display/mcu/mcu`

Big picture:

An MCU is itself a CANopen node but also consumes PDOs from another node. In
the current implementation, the display OD maps remote sensor TPDO COB-IDs as
RPDOs into its local OD, then the application reads local OD state to drive the
display.

Current pain:

- The app needs node and master-ish behavior on one CAN bus.
- Today this works by configuring RPDO COB-IDs to remote TPDO IDs, but the bus
sharing model is implicit.
- The application also wants CAN health / last-frame / last-RPDO timing.

Example to publish:

`examples/embassy-display-consumes-tpdo`

Sketch:

```rust
object_dictionary! {
    pub struct DisplayOd {
        [0x6000] remote_sensor: record {
            [1] h2_ppm: u32 = 0, rw, pdo;
            [2] h2_lel_x100: u16 = 0, rw, pdo;
            [3] status: u8 = 0xff, rw, pdo;
        };

        rpdo[1](cob_id = 0x181, transmission_type = 255) {
            h2_ppm,
            h2_lel_x100,
            status,
        };
    }
}

#[embassy_executor::task]
async fn display_task() {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        ticker.next().await;
        NODE.read(|od| {
            render(od.h2_ppm, od.h2_lel_x100, od.status);
        });
    }
}
```

API pressure:

- document RPDO-as-remote-TPDO-consumer pattern;
- mixed-role bus model in docs;
- scan/read helpers after `SharedNode` refactor;
- health/timeout helpers for "no recent RPDO".

## Use case 6: std simulator for a CANopen node

Representative project:

- `h2/msr-polyguard-h2-can-firmware/simulator`

Big picture:

A Linux process simulates the same `Node<OD>` as embedded firmware against
SocketCAN or SLCAN. It is useful for UI/gateway development and CI.

Current pain:

- Example exists locally but not in the crate.
- Repeats the "run `Node::process()` in a loop, mutate OD periodically"
  pattern.

Example to publish:

`canopen-linux/examples/simulated_sensor_node.rs`

Sketch:

```rust
fn main() -> anyhow::Result<()> {
    let mut transport = SocketcanTransport::open("vcan0")?;
    let clock = StdClock::new();
    let node_id = NodeId::new(1).unwrap();
    let od = SensorOd::new();
    let mut node = Node::new(NodeConfig::from_od(&od, node_id), od);

    loop {
        node.process(&mut transport, &clock);

        if sample_due() {
            let mut od = node.od_mut();
            od.value = simulated_value();
            od.status = 0;
        }

        std::thread::sleep(Duration::from_millis(1));
    }
}
```

API pressure:

- std clock helper;
- `NodeConfig::from_od` docs;
- simulator-friendly transport setup;
- examples should run on `vcan`.

## Use case 7: decoded TPDO/RPDO bridge without full OD

Representative project:

- `frost-res-ros2-gateway`

Big picture:

A gateway does not need a local CANopen OD or SDO. It decodes a few known PDO
layouts from remote nodes and sends simple RPDO control frames.

Current pain:

- The app hand-decodes COB-ID ranges and EMCY payloads.
- It would benefit from `CobId`/`CanOpenEvent` helpers but not a full node.

Example to publish:

`canopen-linux/examples/pdo_bridge.rs`

Sketch:

```rust
fn main() -> anyhow::Result<()> {
    let bus = CanOpenBus::spawn(SocketcanTransport::open("can0")?);
    let mut events = bus.subscribe_canopen::<64>();

    loop {
        match events.recv()? {
            CanOpenEvent::Tpdo { node, pdo_num: 0, frame } => {
                let statusword = u16::from_le_bytes([frame.data()[0], frame.data()[1]]);
                publish_status(node, statusword);
            }
            CanOpenEvent::Emcy(msg) => publish_fault(msg),
            CanOpenEvent::Heartbeat { node, state } => publish_alive(node, state),
            _ => {}
        }

        if let Some(cmd) = next_control_command() {
            bus.send(make_rpdo1(cmd.node, cmd.controlword))?;
        }
    }
}
```

API pressure:

- lightweight CANopen event decoder;
- raw RPDO construction/transmit helpers;
- no requirement to define an OD.

## Suggested example set

Minimum set to cover the real consumer-facing API:

1. `simulated_sensor_node.rs` — std `Node<OD>` on vcan.
2. `embassy_sensor_node` — no_std node publishing TPDOs.
3. `embassy_display_consumes_tpdo` — mixed-role node consuming remote TPDOs.
4. `can_monitor_with_sdo.rs` — std raw/decode monitor plus SDO mailbox.
5. `cia402_velocity_controller.rs` — typed EDS SDO client + PDO command/feedback.
6. `sdo_gateway.rs` — blocking std SDO/NMT service API.
7. `pdo_bridge.rs` — lightweight decoded PDO/EMCY/heartbeat bridge.

These examples intentionally overlap. The overlap is useful: it should make
API friction obvious when the same bus, subscription, SDO, and decode concepts
appear in several application shapes.
