use crate::od::ObjectDictionary;
use crate::transport::CanFrame;
use heapless::Vec;

/// One PDO mapping entry: which OD entry maps to which bits in the PDO frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct PdoMapping {
    /// OD index
    pub index: u16,
    /// OD subindex
    pub subindex: u8,
    /// Length in bits (must be a multiple of 8 for now)
    pub bit_length: u8,
}

impl PdoMapping {
    /// Decode a CANopen PDO mapping value (0xIIIISSLL).
    pub const fn from_mapping_value(val: u32) -> Self {
        Self {
            index: (val >> 16) as u16,
            subindex: (val >> 8) as u8,
            bit_length: val as u8,
        }
    }

    /// Encode to CANopen PDO mapping value.
    pub const fn to_mapping_value(self) -> u32 {
        (self.index as u32) << 16 | (self.subindex as u32) << 8 | self.bit_length as u32
    }
}

/// Configuration for one TPDO (transmit PDO).
#[derive(Clone, Debug)]
pub struct TpdoConfig<const MAX_MAPPINGS: usize = 8> {
    /// COB-ID for this PDO
    pub cob_id: u16,
    /// Transmission type: 0=sync acyclic, 1-240=sync cyclic, 254/255=event-driven
    pub transmission_type: u8,
    /// Inhibit time in units of 100us
    pub inhibit_time_100us: u16,
    /// Event timer in ms (0=disabled)
    pub event_timer_ms: u16,
    /// PDO mappings
    pub mappings: Vec<PdoMapping, MAX_MAPPINGS>,
    /// Whether this PDO is enabled
    pub enabled: bool,
}

impl<const N: usize> Default for TpdoConfig<N> {
    fn default() -> Self {
        Self {
            cob_id: 0,
            transmission_type: 255,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings: Vec::new(),
            enabled: false,
        }
    }
}

/// Configuration for one RPDO (receive PDO).
#[derive(Clone, Debug)]
pub struct RpdoConfig<const MAX_MAPPINGS: usize = 8> {
    /// COB-ID for this PDO
    pub cob_id: u16,
    /// Transmission type
    pub transmission_type: u8,
    /// PDO mappings
    pub mappings: Vec<PdoMapping, MAX_MAPPINGS>,
    /// Whether this PDO is enabled
    pub enabled: bool,
}

impl<const N: usize> Default for RpdoConfig<N> {
    fn default() -> Self {
        Self {
            cob_id: 0,
            transmission_type: 255,
            mappings: Vec::new(),
            enabled: false,
        }
    }
}

/// TPDO engine. Manages up to N transmit PDOs.
pub struct TpdoEngine<const N: usize = 4> {
    pdos: [TpdoConfig; N],
    sync_counter: [u8; N],
    last_send_us: [u64; N],
}

impl<const N: usize> TpdoEngine<N> {
    pub fn new(pdos: [TpdoConfig; N]) -> Self {
        Self {
            pdos,
            sync_counter: [0; N],
            last_send_us: [0; N],
        }
    }

    pub fn config(&self, index: usize) -> Option<&TpdoConfig> {
        self.pdos.get(index)
    }

    pub fn config_mut(&mut self, index: usize) -> Option<&mut TpdoConfig> {
        self.pdos.get_mut(index)
    }

    /// Called on SYNC reception. Returns frames to transmit for sync-triggered PDOs.
    pub fn on_sync<OD: ObjectDictionary>(
        &mut self,
        od: &OD,
        out: &mut Vec<CanFrame, N>,
    ) {
        for i in 0..N {
            if !self.pdos[i].enabled {
                continue;
            }
            let tt = self.pdos[i].transmission_type;
            if tt == 0 {
                // Sync acyclic - only send if triggered (TODO: trigger mechanism)
                continue;
            }
            if tt >= 1 && tt <= 240 {
                // Sync cyclic - send every `tt` SYNCs
                self.sync_counter[i] += 1;
                if self.sync_counter[i] >= tt {
                    self.sync_counter[i] = 0;
                    if let Some(frame) = self.build_pdo_frame(i, od) {
                        let _ = out.push(frame);
                    }
                }
            }
            // 254/255 are event-driven, handled by poll()
        }
    }

    /// Called periodically. Checks event timers for event-driven PDOs.
    pub fn poll<OD: ObjectDictionary>(
        &mut self,
        od: &OD,
        now_us: u64,
        out: &mut Vec<CanFrame, N>,
    ) {
        for i in 0..N {
            if !self.pdos[i].enabled {
                continue;
            }
            let tt = self.pdos[i].transmission_type;
            if (tt == 254 || tt == 255) && self.pdos[i].event_timer_ms > 0 {
                let interval_us = self.pdos[i].event_timer_ms as u64 * 1000;
                if now_us.wrapping_sub(self.last_send_us[i]) >= interval_us {
                    self.last_send_us[i] = now_us;
                    if let Some(frame) = self.build_pdo_frame(i, od) {
                        let _ = out.push(frame);
                    }
                }
            }
        }
    }

    fn build_pdo_frame<OD: ObjectDictionary>(&self, pdo_idx: usize, od: &OD) -> Option<CanFrame> {
        let pdo = &self.pdos[pdo_idx];
        let mut data = [0u8; 8];
        let mut bit_offset: usize = 0;

        for mapping in &pdo.mappings {
            let byte_offset = bit_offset / 8;
            let byte_len = mapping.bit_length as usize / 8;
            if byte_offset + byte_len > 8 {
                return None; // PDO too long
            }
            if od
                .read(
                    mapping.index,
                    mapping.subindex,
                    &mut data[byte_offset..byte_offset + byte_len],
                )
                .is_err()
            {
                return None;
            }
            bit_offset += mapping.bit_length as usize;
        }

        let total_bytes = (bit_offset + 7) / 8;
        CanFrame::new(pdo.cob_id, &data[..total_bytes])
    }
}

/// RPDO engine. Manages up to N receive PDOs.
pub struct RpdoEngine<const N: usize = 4> {
    pdos: [RpdoConfig; N],
}

impl<const N: usize> RpdoEngine<N> {
    pub fn new(pdos: [RpdoConfig; N]) -> Self {
        Self { pdos }
    }

    pub fn config(&self, index: usize) -> Option<&RpdoConfig> {
        self.pdos.get(index)
    }

    pub fn config_mut(&mut self, index: usize) -> Option<&mut RpdoConfig> {
        self.pdos.get_mut(index)
    }

    /// Process an incoming CAN frame. If it matches an RPDO, write mapped values to OD.
    /// Returns true if the frame was consumed by an RPDO.
    pub fn process<OD: ObjectDictionary>(
        &self,
        frame: &CanFrame,
        od: &mut OD,
    ) -> bool {
        for pdo in &self.pdos {
            if !pdo.enabled || frame.id() != pdo.cob_id {
                continue;
            }

            let data = frame.data();
            let mut bit_offset: usize = 0;

            for mapping in &pdo.mappings {
                let byte_offset = bit_offset / 8;
                let byte_len = mapping.bit_length as usize / 8;
                if byte_offset + byte_len > data.len() {
                    break;
                }
                let _ = od.write(
                    mapping.index,
                    mapping.subindex,
                    &data[byte_offset..byte_offset + byte_len],
                );
                bit_offset += mapping.bit_length as usize;
            }
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::od::*;
    use crate::datatypes::DataType;

    struct PdoTestOd {
        input1: u8,
        input2: u16,
        output1: u8,
    }

    static PDO_TEST_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x6000, subindex: 1, data_type: DataType::U8,
            access: AccessType::Ro, pdo_mappable: true, name: "input1",
        },
        OdEntryMeta {
            index: 0x6000, subindex: 2, data_type: DataType::U16,
            access: AccessType::Ro, pdo_mappable: true, name: "input2",
        },
        OdEntryMeta {
            index: 0x6200, subindex: 1, data_type: DataType::U8,
            access: AccessType::Rw, pdo_mappable: true, name: "output1",
        },
    ];

    impl ObjectDictionary for PdoTestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            PDO_TEST_META.iter().find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x6000, 1) => { buf[0] = self.input1; Ok(1) }
                (0x6000, 2) => { buf[..2].copy_from_slice(&self.input2.to_le_bytes()); Ok(2) }
                (0x6200, 1) => { buf[0] = self.output1; Ok(1) }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x6200, 1) => { self.output1 = data[0]; Ok(()) }
                _ => Err(OdError::ReadOnly),
            }
        }
        fn sub_count(&self, _index: u16) -> Option<u8> { None }
    }

    #[test]
    fn tpdo_sync_cyclic() {
        let mut mappings = Vec::<PdoMapping, 8>::new();
        mappings.push(PdoMapping { index: 0x6000, subindex: 1, bit_length: 8 }).unwrap();
        mappings.push(PdoMapping { index: 0x6000, subindex: 2, bit_length: 16 }).unwrap();

        let config = TpdoConfig {
            cob_id: 0x181,
            transmission_type: 1, // every SYNC
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        };

        let mut engine = TpdoEngine::new([config]);
        let od = PdoTestOd { input1: 0x42, input2: 0x1234, output1: 0 };
        let mut out = Vec::<CanFrame, 1>::new();

        engine.on_sync(&od, &mut out);
        assert_eq!(out.len(), 1);

        let frame = &out[0];
        assert_eq!(frame.id(), 0x181);
        assert_eq!(frame.data(), &[0x42, 0x34, 0x12]); // u8 + u16 LE
    }

    #[test]
    fn rpdo_process() {
        let mut mappings = Vec::<PdoMapping, 8>::new();
        mappings.push(PdoMapping { index: 0x6200, subindex: 1, bit_length: 8 }).unwrap();

        let config = RpdoConfig {
            cob_id: 0x201,
            transmission_type: 255,
            mappings,
            enabled: true,
        };

        let engine = RpdoEngine::new([config]);
        let mut od = PdoTestOd { input1: 0, input2: 0, output1: 0 };

        let frame = CanFrame::new(0x201, &[0xFF]).unwrap();
        assert!(engine.process(&frame, &mut od));
        assert_eq!(od.output1, 0xFF);

        // Non-matching frame
        let frame2 = CanFrame::new(0x301, &[0x00]).unwrap();
        assert!(!engine.process(&frame2, &mut od));
        assert_eq!(od.output1, 0xFF); // unchanged
    }

    #[test]
    fn tpdo_event_timer() {
        let mut mappings = Vec::<PdoMapping, 8>::new();
        mappings.push(PdoMapping { index: 0x6000, subindex: 1, bit_length: 8 }).unwrap();

        let config = TpdoConfig {
            cob_id: 0x181,
            transmission_type: 255, // event-driven
            inhibit_time_100us: 0,
            event_timer_ms: 100, // every 100ms
            mappings,
            enabled: true,
        };

        let mut engine = TpdoEngine::new([config]);
        let od = PdoTestOd { input1: 0x99, input2: 0, output1: 0 };

        // At t=0, diff is 0 which is < 100ms
        let mut out = Vec::<CanFrame, 1>::new();
        engine.poll(&od, 0, &mut out);
        assert_eq!(out.len(), 0);

        // At t=100ms - first send
        engine.poll(&od, 100_000, &mut out);
        assert_eq!(out.len(), 1);

        // 150ms later - too early
        out.clear();
        engine.poll(&od, 150_000, &mut out);
        assert_eq!(out.len(), 0);

        // 200ms - due again
        out.clear();
        engine.poll(&od, 200_000, &mut out);
        assert_eq!(out.len(), 1);
    }
}
