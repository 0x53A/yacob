use crate::od::{ObjectDictionary, OdError};
use crate::sdo::protocol::*;

/// State for a segmented transfer in progress.
struct SegmentedTransfer {
    index: u16,
    subindex: u8,
    toggle: bool,
    direction: Direction,
    buf: [u8; 256],
    offset: usize,
    total_len: usize,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Upload,   // reading from OD, sending to client
    Download, // receiving from client, writing to OD
}

/// SDO server state machine. Handles one transfer at a time.
pub struct SdoServer {
    transfer: Option<SegmentedTransfer>,
}

impl SdoServer {
    pub const fn new() -> Self {
        Self { transfer: None }
    }

    /// Process an incoming SDO request frame and produce a response.
    ///
    /// Returns `Ok(())` if a response was written to `response`.
    /// Returns `Err(())` if the frame was malformed and no response should be sent.
    pub fn process<OD: ObjectDictionary>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
    ) -> Result<(), ()> {
        let cs = command_specifier(request[0]);

        match cs {
            cs if cs == Ccs::InitiateUpload as u8 => {
                self.handle_initiate_upload(request, od, response)
            }
            cs if cs == Ccs::UploadSegment as u8 => {
                self.handle_upload_segment(request, response)
            }
            cs if cs == Ccs::InitiateDownload as u8 => {
                self.handle_initiate_download(request, od, response)
            }
            cs if cs == Ccs::DownloadSegment as u8 => {
                self.handle_download_segment(request, od, response)
            }
            cs if cs == Ccs::AbortTransfer as u8 => {
                self.transfer = None;
                Err(()) // no response for abort from client
            }
            _ => {
                let (index, subindex) = parse_index_sub(request);
                *response = encode_abort(index, subindex, AbortCode::InvalidCommandSpecifier);
                Ok(())
            }
        }
    }

    fn handle_initiate_upload<OD: ObjectDictionary>(
        &mut self,
        request: &[u8; 8],
        od: &OD,
        response: &mut [u8; 8],
    ) -> Result<(), ()> {
        let (index, subindex) = parse_index_sub(request);

        // Read value from OD
        let mut buf = [0u8; 256];
        let len = match od.read(index, subindex, &mut buf) {
            Ok(n) => n,
            Err(e) => {
                *response = encode_abort(index, subindex, od_error_to_abort(e));
                return Ok(());
            }
        };

        if len <= 4 {
            // Expedited transfer
            *response =
                encode_upload_response_expedited(index, subindex, &buf[..len]).unwrap();
            self.transfer = None;
        } else {
            // Segmented transfer
            *response = encode_upload_response_segmented(index, subindex, len as u32);
            self.transfer = Some(SegmentedTransfer {
                index,
                subindex,
                toggle: false,
                direction: Direction::Upload,
                buf,
                offset: 0,
                total_len: len,
            });
        }
        Ok(())
    }

    fn handle_upload_segment(
        &mut self,
        request: &[u8; 8],
        response: &mut [u8; 8],
    ) -> Result<(), ()> {
        let transfer = match &mut self.transfer {
            Some(t) if t.direction == Direction::Upload => t,
            _ => {
                // No active upload transfer
                *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                return Ok(());
            }
        };

        // Check toggle bit
        let client_toggle = (request[0] & 0x10) != 0;
        if client_toggle != transfer.toggle {
            let (idx, sub) = (transfer.index, transfer.subindex);
            self.transfer = None;
            *response = encode_abort(idx, sub, AbortCode::ToggleBitNotAlternated);
            return Ok(());
        }

        // Send next segment (up to 7 bytes)
        let remaining = transfer.total_len - transfer.offset;
        let seg_len = remaining.min(7);
        let last = remaining <= 7;

        *response = encode_upload_segment_response(
            transfer.toggle,
            &transfer.buf[transfer.offset..transfer.offset + seg_len],
            last,
        );

        transfer.offset += seg_len;
        transfer.toggle = !transfer.toggle;

        if last {
            self.transfer = None;
        }
        Ok(())
    }

    fn handle_initiate_download<OD: ObjectDictionary>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
    ) -> Result<(), ()> {
        let (index, subindex) = parse_index_sub(request);
        let expedited = (request[0] & 0x02) != 0;
        let size_indicated = (request[0] & 0x01) != 0;

        if expedited {
            // Expedited download
            let n = if size_indicated {
                ((request[0] >> 2) & 0x03) as usize
            } else {
                0
            };
            let data_len = 4 - n;
            let data = &request[4..4 + data_len];

            match od.write(index, subindex, data) {
                Ok(()) => {
                    *response = encode_download_response(index, subindex);
                }
                Err(e) => {
                    *response = encode_abort(index, subindex, od_error_to_abort(e));
                }
            }
            self.transfer = None;
        } else {
            // Segmented download - initiate
            let total_len = if size_indicated {
                u32::from_le_bytes([request[4], request[5], request[6], request[7]]) as usize
            } else {
                0
            };

            if total_len > 256 {
                *response = encode_abort(index, subindex, AbortCode::OutOfMemory);
                return Ok(());
            }

            self.transfer = Some(SegmentedTransfer {
                index,
                subindex,
                toggle: false,
                direction: Direction::Download,
                buf: [0u8; 256],
                offset: 0,
                total_len,
            });
            *response = encode_download_response(index, subindex);
        }
        Ok(())
    }

    fn handle_download_segment<OD: ObjectDictionary>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
    ) -> Result<(), ()> {
        let transfer = match &mut self.transfer {
            Some(t) if t.direction == Direction::Download => t,
            _ => {
                *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                return Ok(());
            }
        };

        // Check toggle bit
        let client_toggle = (request[0] & 0x10) != 0;
        if client_toggle != transfer.toggle {
            let (idx, sub) = (transfer.index, transfer.subindex);
            self.transfer = None;
            *response = encode_abort(idx, sub, AbortCode::ToggleBitNotAlternated);
            return Ok(());
        }

        let last = (request[0] & 0x01) != 0;
        let n = ((request[0] >> 1) & 0x07) as usize; // number of unused bytes in segment
        let seg_len = 7 - n;

        // Copy segment data into buffer
        if transfer.offset + seg_len > 256 {
            let (idx, sub) = (transfer.index, transfer.subindex);
            self.transfer = None;
            *response = encode_abort(idx, sub, AbortCode::OutOfMemory);
            return Ok(());
        }

        transfer.buf[transfer.offset..transfer.offset + seg_len]
            .copy_from_slice(&request[1..1 + seg_len]);
        transfer.offset += seg_len;
        transfer.toggle = !transfer.toggle;

        *response = encode_download_segment_response(client_toggle);

        if last {
            let index = transfer.index;
            let subindex = transfer.subindex;
            let data_len = transfer.offset;
            let buf_copy: [u8; 256] = transfer.buf;
            self.transfer = None;

            match od.write(index, subindex, &buf_copy[..data_len]) {
                Ok(()) => {} // response already set
                Err(e) => {
                    *response = encode_abort(index, subindex, od_error_to_abort(e));
                }
            }
        }

        Ok(())
    }
}

fn od_error_to_abort(e: OdError) -> AbortCode {
    match e {
        OdError::NotFound => AbortCode::ObjectNotFound,
        OdError::ReadOnly => AbortCode::ReadOnlyObject,
        OdError::WriteOnly => AbortCode::WriteOnlyObject,
        OdError::DataTypeMismatch => AbortCode::DataTypeMismatch,
        OdError::ValueTooLong => AbortCode::OutOfMemory,
        OdError::ValueRange => AbortCode::ValueRangeExceeded,
        OdError::HardwareError => AbortCode::GeneralError,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::od::*;
    use crate::datatypes::DataType;

    /// Simple test OD with a few entries.
    struct TestOd {
        device_type: u32,
        error_register: u8,
        writable_u16: u16,
        long_data: [u8; 20],
    }

    impl TestOd {
        fn new() -> Self {
            Self {
                device_type: 0x0000_0191,
                error_register: 0,
                writable_u16: 0x1234,
                long_data: [0xAA; 20],
            }
        }
    }

    static TEST_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x1000, subindex: 0, data_type: DataType::U32,
            access: AccessType::Ro, pdo_mappable: false, name: "device_type",
        },
        OdEntryMeta {
            index: 0x1001, subindex: 0, data_type: DataType::U8,
            access: AccessType::Ro, pdo_mappable: false, name: "error_register",
        },
        OdEntryMeta {
            index: 0x2000, subindex: 0, data_type: DataType::U16,
            access: AccessType::Rw, pdo_mappable: false, name: "writable_u16",
        },
        OdEntryMeta {
            index: 0x2001, subindex: 0, data_type: DataType::OctetString,
            access: AccessType::Rw, pdo_mappable: false, name: "long_data",
        },
    ];

    impl ObjectDictionary for TestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            TEST_META.iter().find(|e| e.index == index && e.subindex == subindex)
        }

        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1000, 0) => {
                    buf[..4].copy_from_slice(&self.device_type.to_le_bytes());
                    Ok(4)
                }
                (0x1001, 0) => {
                    buf[0] = self.error_register;
                    Ok(1)
                }
                (0x2000, 0) => {
                    buf[..2].copy_from_slice(&self.writable_u16.to_le_bytes());
                    Ok(2)
                }
                (0x2001, 0) => {
                    buf[..20].copy_from_slice(&self.long_data);
                    Ok(20)
                }
                _ => Err(OdError::NotFound),
            }
        }

        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x1000, 0) | (0x1001, 0) => Err(OdError::ReadOnly),
                (0x2000, 0) => {
                    if data.len() != 2 {
                        return Err(OdError::DataTypeMismatch);
                    }
                    self.writable_u16 = u16::from_le_bytes([data[0], data[1]]);
                    Ok(())
                }
                (0x2001, 0) => {
                    if data.len() > 20 {
                        return Err(OdError::ValueTooLong);
                    }
                    self.long_data[..data.len()].copy_from_slice(data);
                    Ok(())
                }
                _ => Err(OdError::NotFound),
            }
        }

        fn sub_count(&self, _index: u16) -> Option<u8> {
            Some(0)
        }
    }

    #[test]
    fn expedited_upload() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Initiate upload for 0x1000:0 (device_type, u32)
        let req = [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]; // CCS=2
        server.process(&req, &mut od, &mut resp).unwrap();

        // Should be expedited response with 4 bytes
        assert_eq!(command_specifier(resp[0]), Scs::InitiateUploadResponse as u8);
        assert!(resp[0] & 0x02 != 0); // expedited
        assert_eq!(resp[4..8], 0x0000_0191u32.to_le_bytes());
    }

    #[test]
    fn expedited_download() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Write 0xABCD to 0x2000:0
        // CCS=1, n=2 (2 unused bytes), e=1, s=1
        let req = [0x2B, 0x00, 0x20, 0x00, 0xCD, 0xAB, 0x00, 0x00];
        server.process(&req, &mut od, &mut resp).unwrap();

        assert_eq!(command_specifier(resp[0]), Scs::InitiateDownloadResponse as u8);
        assert_eq!(od.writable_u16, 0xABCD);
    }

    #[test]
    fn upload_read_only_write_rejected() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Try to write to 0x1000:0 (read-only)
        let req = [0x23, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00];
        server.process(&req, &mut od, &mut resp).unwrap();

        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        assert_eq!(code, AbortCode::ReadOnlyObject as u32);
    }

    #[test]
    fn object_not_found() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        let req = [0x40, 0xFF, 0xFF, 0x00, 0, 0, 0, 0];
        server.process(&req, &mut od, &mut resp).unwrap();

        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        assert_eq!(code, AbortCode::ObjectNotFound as u32);
    }

    #[test]
    fn segmented_upload() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Initiate upload for 0x2001:0 (20-byte octet string)
        let req = [0x40, 0x01, 0x20, 0x00, 0, 0, 0, 0];
        server.process(&req, &mut od, &mut resp).unwrap();

        // Should be segmented initiate response
        assert_eq!(command_specifier(resp[0]), Scs::InitiateUploadResponse as u8);
        assert_eq!(resp[0] & 0x02, 0); // not expedited
        let size = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        assert_eq!(size, 20);

        // Read segments
        let mut all_data = [0u8; 20];
        let mut offset = 0;
        let mut toggle = false;

        loop {
            let seg_req = [
                (Ccs::UploadSegment as u8) << 5 | if toggle { 0x10 } else { 0 },
                0, 0, 0, 0, 0, 0, 0,
            ];
            server.process(&seg_req, &mut od, &mut resp).unwrap();

            let scs = command_specifier(resp[0]);
            assert_eq!(scs, Scs::UploadSegmentResponse as u8);

            let n = ((resp[0] >> 1) & 0x07) as usize;
            let last = (resp[0] & 0x01) != 0;
            let seg_len = 7 - n;

            all_data[offset..offset + seg_len].copy_from_slice(&resp[1..1 + seg_len]);
            offset += seg_len;
            toggle = !toggle;

            if last {
                break;
            }
        }

        assert_eq!(offset, 20);
        assert_eq!(all_data, [0xAA; 20]);
    }
}
