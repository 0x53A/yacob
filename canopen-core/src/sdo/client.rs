use crate::cobid::{CobId, NodeId};
use crate::sdo::protocol::*;
use crate::transport::CanFrame;

/// Result of processing an SDO response in the client.
pub enum SdoClientResult {
    /// Upload (read) transfer complete. Data is in the client's buffer.
    UploadComplete { data_len: usize },
    /// Download (write) transfer acknowledged by server.
    DownloadComplete,
    /// Segmented transfer: send this frame next.
    SendNext(CanFrame),
    /// Transfer aborted by server.
    Aborted(AbortCode),
    /// Abort for an unrelated index/subindex while a transfer is active
    /// (e.g. another client's rejected request on the same SDO channel).
    /// The active transfer continues; keep waiting for the real response.
    IgnoredAbort,
    /// Protocol error (unexpected response).
    Error,
}

enum State {
    Idle,
    WaitingUploadInitResponse {
        index: u16,
        subindex: u8,
    },
    UploadSegmented {
        index: u16,
        subindex: u8,
        toggle: bool,
    },
    WaitingDownloadInitResponse {
        index: u16,
        subindex: u8,
    },
    DownloadSegmented {
        index: u16,
        subindex: u8,
        toggle: bool,
    },
}

/// SDO client for talking to a remote CANopen node.
///
/// Tracks one active transfer at a time. The user drives the transfer loop:
/// 1. Call `start_upload()` or `start_download()` to get the first frame to send.
/// 2. Send the frame via transport.
/// 3. When a response arrives, call `process_response()`.
/// 4. If it returns `SendNext(frame)`, send that frame and repeat from 3.
/// 5. `UploadComplete` or `DownloadComplete` signals success.
///
/// The `BUF` const generic controls the maximum segmented transfer size.
/// Default is 256 bytes. Use `SdoClient<889>` for max CiA 301 block size.
pub struct SdoClient<const BUF: usize = 256> {
    target_node: NodeId,
    state: State,
    /// Shared buffer: holds received data during upload, or data to send during download.
    buf: [u8; BUF],
    offset: usize,
    total_len: usize,
}

impl<const BUF: usize> SdoClient<BUF> {
    pub fn new(target: NodeId) -> Self {
        Self {
            target_node: target,
            state: State::Idle,
            buf: [0; BUF],
            offset: 0,
            total_len: 0,
        }
    }

    /// Access the data buffer after an UploadComplete.
    pub fn data(&self) -> &[u8] {
        &self.buf[..self.offset]
    }

    /// The COB-ID for SDO requests to this node (client→server).
    fn tx_cobid(&self) -> u16 {
        CobId::sdo_rx(self.target_node).raw()
    }

    /// Start an upload (read) transfer. Returns the initiate request frame to send.
    pub fn start_upload(&mut self, index: u16, subindex: u8) -> CanFrame {
        self.state = State::WaitingUploadInitResponse { index, subindex };
        self.offset = 0;
        self.total_len = 0;

        let mut data = [0u8; 8];
        data[0] = (Ccs::InitiateUpload as u8) << 5; // 0x40
        data[1] = (index & 0xFF) as u8;
        data[2] = (index >> 8) as u8;
        data[3] = subindex;
        CanFrame::new(self.tx_cobid(), &data).unwrap()
    }

    /// Start a download (write) transfer. Returns the initiate request frame
    /// to send, or `Err(())` if `value` does not fit the client's `BUF`-byte
    /// transfer buffer (nothing is sent; the client stays idle).
    pub fn start_download(
        &mut self,
        index: u16,
        subindex: u8,
        value: &[u8],
    ) -> Result<CanFrame, ()> {
        if value.len() <= 4 {
            // Expedited download
            self.state = State::WaitingDownloadInitResponse { index, subindex };
            self.total_len = value.len();
            let n = 4 - value.len();
            let mut data = [0u8; 8];
            // CCS=1, n, e=1, s=1
            data[0] = (Ccs::InitiateDownload as u8) << 5 | (n as u8) << 2 | 0x02 | 0x01;
            data[1] = (index & 0xFF) as u8;
            data[2] = (index >> 8) as u8;
            data[3] = subindex;
            data[4..4 + value.len()].copy_from_slice(value);
            Ok(CanFrame::new(self.tx_cobid(), &data).unwrap())
        } else {
            if value.len() > BUF {
                self.state = State::Idle;
                self.total_len = 0;
                self.offset = 0;
                return Err(());
            }

            // Segmented download — store data in shared buffer
            self.state = State::WaitingDownloadInitResponse { index, subindex };
            self.buf[..value.len()].copy_from_slice(value);
            self.total_len = value.len();
            self.offset = 0;

            let mut data = [0u8; 8];
            // CCS=1, e=0, s=1
            data[0] = (Ccs::InitiateDownload as u8) << 5 | 0x01;
            data[1] = (index & 0xFF) as u8;
            data[2] = (index >> 8) as u8;
            data[3] = subindex;
            data[4..8].copy_from_slice(&(value.len() as u32).to_le_bytes());
            Ok(CanFrame::new(self.tx_cobid(), &data).unwrap())
        }
    }

    /// Process an SDO response from the server.
    pub fn process_response(&mut self, response: &[u8; 8]) -> SdoClientResult {
        let cs = command_specifier(response[0]);

        // Check for abort. While a transfer is active, only an abort
        // addressing the active object — or the channel-level 0x0000:00 —
        // cancels it; aborts for unrelated objects (another client's
        // rejected request on the same channel) are ignored.
        if cs == Scs::AbortTransfer as u8 {
            let abort_index = u16::from_le_bytes([response[1], response[2]]);
            let abort_sub = response[3];
            let active = match &self.state {
                State::Idle => None,
                State::WaitingUploadInitResponse { index, subindex }
                | State::WaitingDownloadInitResponse { index, subindex } => {
                    Some((*index, *subindex))
                }
                State::UploadSegmented {
                    index, subindex, ..
                }
                | State::DownloadSegmented {
                    index, subindex, ..
                } => Some((*index, *subindex)),
            };
            if let Some((index, subindex)) = active {
                let channel_level = abort_index == 0 && abort_sub == 0;
                if !channel_level && (abort_index, abort_sub) != (index, subindex) {
                    return SdoClientResult::IgnoredAbort;
                }
            }
            self.state = State::Idle;
            let code = u32::from_le_bytes([response[4], response[5], response[6], response[7]]);
            // Find matching abort code
            return SdoClientResult::Aborted(match code {
                0x0503_0000 => AbortCode::ToggleBitNotAlternated,
                0x0504_0000 => AbortCode::SdoProtocolTimeout,
                0x0504_0001 => AbortCode::InvalidCommandSpecifier,
                0x0504_0002 => AbortCode::InvalidBlockSize,
                0x0504_0003 => AbortCode::InvalidSequenceNumber,
                0x0504_0005 => AbortCode::OutOfMemory,
                0x0601_0000 => AbortCode::UnsupportedAccess,
                0x0601_0001 => AbortCode::WriteOnlyObject,
                0x0601_0002 => AbortCode::ReadOnlyObject,
                0x0602_0000 => AbortCode::ObjectNotFound,
                0x0604_0041 => AbortCode::ObjectCannotBeMapped,
                0x0604_0042 => AbortCode::PdoLengthExceeded,
                0x0604_0043 => AbortCode::ParameterIncompatibility,
                0x0609_0011 => AbortCode::SubindexNotFound,
                0x0609_0030 => AbortCode::ValueRangeExceeded,
                0x0609_0031 => AbortCode::ValueTooHigh,
                0x0609_0032 => AbortCode::ValueTooLow,
                0x0609_0043 => AbortCode::DataTypeMismatch,
                0x0800_0020 => AbortCode::DataTransferError,
                0x0800_0021 => AbortCode::DataTransferLocalControl,
                0x0800_0022 => AbortCode::DataTransferDeviceState,
                _ => AbortCode::GeneralError,
            });
        }

        match &self.state {
            State::Idle => SdoClientResult::Error,

            State::WaitingUploadInitResponse { .. } => {
                if cs != Scs::InitiateUploadResponse as u8 {
                    self.state = State::Idle;
                    return SdoClientResult::Error;
                }
                self.handle_upload_init_response(response)
            }

            State::UploadSegmented { .. } => {
                if cs != Scs::UploadSegmentResponse as u8 {
                    self.state = State::Idle;
                    return SdoClientResult::Error;
                }
                self.handle_upload_segment_response(response)
            }

            State::WaitingDownloadInitResponse { .. } => {
                if cs != Scs::InitiateDownloadResponse as u8 {
                    self.state = State::Idle;
                    return SdoClientResult::Error;
                }
                self.handle_download_init_response(response)
            }

            State::DownloadSegmented { .. } => {
                if cs != Scs::DownloadSegmentResponse as u8 {
                    self.state = State::Idle;
                    return SdoClientResult::Error;
                }
                self.handle_download_segment_response(response)
            }
        }
    }

    fn handle_upload_init_response(&mut self, response: &[u8; 8]) -> SdoClientResult {
        let expedited = (response[0] & 0x02) != 0;
        let size_indicated = (response[0] & 0x01) != 0;

        if expedited {
            let n = if size_indicated {
                ((response[0] >> 2) & 0x03) as usize
            } else {
                0
            };
            let data_len = 4 - n;
            self.buf[..data_len].copy_from_slice(&response[4..4 + data_len]);
            self.offset = data_len;
            self.state = State::Idle;
            SdoClientResult::UploadComplete { data_len }
        } else {
            // Segmented transfer
            if size_indicated {
                self.total_len =
                    u32::from_le_bytes([response[4], response[5], response[6], response[7]])
                        as usize;
            }

            let (index, subindex) = match &self.state {
                State::WaitingUploadInitResponse { index, subindex } => (*index, *subindex),
                _ => unreachable!(),
            };

            self.state = State::UploadSegmented {
                index,
                subindex,
                toggle: false,
            };
            self.offset = 0;

            // Send first segment request
            let frame = self.make_upload_segment_request(false);
            SdoClientResult::SendNext(frame)
        }
    }

    fn handle_upload_segment_response(&mut self, response: &[u8; 8]) -> SdoClientResult {
        let (index, subindex, expected_toggle) = match &self.state {
            State::UploadSegmented {
                index,
                subindex,
                toggle,
            } => (*index, *subindex, *toggle),
            _ => return SdoClientResult::Error,
        };

        let server_toggle = (response[0] & 0x10) != 0;
        if server_toggle != expected_toggle {
            self.state = State::Idle;
            return SdoClientResult::Error;
        }

        let n = ((response[0] >> 1) & 0x07) as usize;
        let last = (response[0] & 0x01) != 0;
        let seg_len = 7 - n;

        if self.offset + seg_len > BUF {
            self.state = State::Idle;
            return SdoClientResult::Error;
        }

        self.buf[self.offset..self.offset + seg_len].copy_from_slice(&response[1..1 + seg_len]);
        self.offset += seg_len;

        if last {
            let data_len = self.offset;
            self.state = State::Idle;
            SdoClientResult::UploadComplete { data_len }
        } else {
            let new_toggle = !expected_toggle;
            self.state = State::UploadSegmented {
                index,
                subindex,
                toggle: new_toggle,
            };
            let frame = self.make_upload_segment_request(new_toggle);
            SdoClientResult::SendNext(frame)
        }
    }

    fn handle_download_init_response(&mut self, _response: &[u8; 8]) -> SdoClientResult {
        if self.total_len <= 4 {
            // Was expedited, download is done
            self.state = State::Idle;
            SdoClientResult::DownloadComplete
        } else {
            // Start sending segments
            let (index, subindex) = match &self.state {
                State::WaitingDownloadInitResponse { index, subindex } => (*index, *subindex),
                _ => unreachable!(),
            };
            self.state = State::DownloadSegmented {
                index,
                subindex,
                toggle: false,
            };
            let frame = self.make_download_segment(false);
            SdoClientResult::SendNext(frame)
        }
    }

    fn handle_download_segment_response(&mut self, response: &[u8; 8]) -> SdoClientResult {
        let (index, subindex, expected_toggle) = match &self.state {
            State::DownloadSegmented {
                index,
                subindex,
                toggle,
            } => (*index, *subindex, *toggle),
            _ => return SdoClientResult::Error,
        };

        let server_toggle = (response[0] & 0x10) != 0;
        if server_toggle != expected_toggle {
            self.state = State::Idle;
            return SdoClientResult::Error;
        }

        if self.offset >= self.total_len {
            // All data sent and acknowledged
            self.state = State::Idle;
            SdoClientResult::DownloadComplete
        } else {
            let new_toggle = !expected_toggle;
            self.state = State::DownloadSegmented {
                index,
                subindex,
                toggle: new_toggle,
            };
            let frame = self.make_download_segment(new_toggle);
            SdoClientResult::SendNext(frame)
        }
    }

    fn make_upload_segment_request(&self, toggle: bool) -> CanFrame {
        let mut data = [0u8; 8];
        data[0] = (Ccs::UploadSegment as u8) << 5 | if toggle { 0x10 } else { 0 };
        CanFrame::new(self.tx_cobid(), &data).unwrap()
    }

    fn make_download_segment(&mut self, toggle: bool) -> CanFrame {
        let remaining = self.total_len - self.offset;
        let seg_len = remaining.min(7);
        let last = remaining <= 7;
        let n = 7 - seg_len;

        let mut data = [0u8; 8];
        data[0] = (Ccs::DownloadSegment as u8) << 5
            | if toggle { 0x10 } else { 0 }
            | (n as u8) << 1
            | if last { 0x01 } else { 0 };
        data[1..1 + seg_len].copy_from_slice(&self.buf[self.offset..self.offset + seg_len]);
        self.offset += seg_len;

        CanFrame::new(self.tx_cobid(), &data).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datatypes::DataType;
    use crate::od::*;
    use crate::sdo::server::SdoServer;

    /// Minimal test OD for client-server tests.
    struct TestOd {
        val_u32: u32,
        val_u16: u16,
        blob: [u8; 20],
    }

    static CLIENT_TEST_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x1000,
            subindex: 0,
            data_type: DataType::U32,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "val_u32",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x2000,
            subindex: 0,
            data_type: DataType::U16,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "val_u16",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x2001,
            subindex: 0,
            data_type: DataType::OctetString,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "blob",
            max_size: None,
        },
    ];

    impl ObjectDictionary for TestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            CLIENT_TEST_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1000, 0) => {
                    buf[..4].copy_from_slice(&self.val_u32.to_le_bytes());
                    Ok(4)
                }
                (0x2000, 0) => {
                    buf[..2].copy_from_slice(&self.val_u16.to_le_bytes());
                    Ok(2)
                }
                (0x2001, 0) => {
                    buf[..20].copy_from_slice(&self.blob);
                    Ok(20)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x1000, 0) => Err(OdError::ReadOnly),
                (0x2000, 0) => {
                    if data.len() != 2 {
                        return Err(OdError::DataTypeMismatch);
                    }
                    self.val_u16 = u16::from_le_bytes([data[0], data[1]]);
                    Ok(())
                }
                (0x2001, 0) => {
                    if data.len() > 20 {
                        return Err(OdError::ValueTooLong);
                    }
                    self.blob = [0; 20];
                    self.blob[..data.len()].copy_from_slice(data);
                    Ok(())
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn sub_count(&self, _index: u16) -> Option<u8> {
            Some(0)
        }
    }

    /// Run a complete SDO transfer between client and server in-memory.
    fn run_transfer(
        client: &mut SdoClient<256>,
        server: &mut SdoServer,
        od: &mut TestOd,
        first_frame: CanFrame,
    ) -> SdoClientResult {
        use crate::nmt::NmtState;
        use crate::od::OdEvent;
        use heapless::Deque;

        // First frame goes from client to server
        let mut req_data = [0u8; 8];
        req_data.copy_from_slice(first_frame.data());

        let mut resp_data = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req_data,
                od,
                &mut resp_data,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();

        let mut result = client.process_response(&resp_data);

        // Loop for segmented transfers
        loop {
            match result {
                SdoClientResult::SendNext(frame) => {
                    req_data.copy_from_slice(frame.data());
                    server
                        .process(
                            &req_data,
                            od,
                            &mut resp_data,
                            &mut events,
                            NmtState::PreOperational,
                            0,
                        )
                        .unwrap();
                    result = client.process_response(&resp_data);
                }
                _ => return result,
            }
        }
    }

    #[test]
    fn client_expedited_upload() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let mut server = SdoServer::new();
        let mut od = TestOd {
            val_u32: 0xDEADBEEF,
            val_u16: 0,
            blob: [0; 20],
        };

        let req = client.start_upload(0x1000, 0);
        match run_transfer(&mut client, &mut server, &mut od, req) {
            SdoClientResult::UploadComplete { data_len } => {
                assert_eq!(data_len, 4);
                assert_eq!(client.data(), &0xDEADBEEFu32.to_le_bytes());
            }
            _ => panic!("expected UploadComplete"),
        }
    }

    #[test]
    fn client_expedited_download() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let mut server = SdoServer::new();
        let mut od = TestOd {
            val_u32: 0,
            val_u16: 0,
            blob: [0; 20],
        };

        let req = client
            .start_download(0x2000, 0, &0xCAFEu16.to_le_bytes())
            .unwrap();
        match run_transfer(&mut client, &mut server, &mut od, req) {
            SdoClientResult::DownloadComplete => {
                assert_eq!(od.val_u16, 0xCAFE);
            }
            _ => panic!("expected DownloadComplete"),
        }
    }

    #[test]
    fn client_segmented_upload() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let mut server = SdoServer::new();
        let mut od = TestOd {
            val_u32: 0,
            val_u16: 0,
            blob: [0xBB; 20],
        };

        let req = client.start_upload(0x2001, 0);
        match run_transfer(&mut client, &mut server, &mut od, req) {
            SdoClientResult::UploadComplete { data_len } => {
                assert_eq!(data_len, 20);
                assert_eq!(client.data(), &[0xBB; 20]);
            }
            _ => panic!("expected UploadComplete"),
        }
    }

    #[test]
    fn client_segmented_download() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let mut server = SdoServer::new();
        let mut od = TestOd {
            val_u32: 0,
            val_u16: 0,
            blob: [0; 20],
        };

        let data: [u8; 20] = [
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
        ];
        let req = client.start_download(0x2001, 0, &data).unwrap();
        match run_transfer(&mut client, &mut server, &mut od, req) {
            SdoClientResult::DownloadComplete => {
                assert_eq!(od.blob, data);
            }
            _ => panic!("expected DownloadComplete"),
        }
    }

    #[test]
    fn client_download_larger_than_buffer_fails_without_sending() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let data = [0x55u8; 257];

        assert!(client.start_download(0x2001, 0, &data).is_err());
        // Client must stay idle so it can be reused.
        let req = client
            .start_download(0x2000, 0, &0xCAFEu16.to_le_bytes())
            .unwrap();
        assert_eq!(req.data()[0] >> 5, Ccs::InitiateDownload as u8);
    }

    /// Build a server abort frame payload for the given object and code.
    fn abort_payload(index: u16, subindex: u8, code: u32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0] = (Scs::AbortTransfer as u8) << 5;
        data[1] = (index & 0xFF) as u8;
        data[2] = (index >> 8) as u8;
        data[3] = subindex;
        data[4..8].copy_from_slice(&code.to_le_bytes());
        data
    }

    #[test]
    fn client_ignores_abort_for_unrelated_object() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let _req = client.start_upload(0x1000, 0);

        // Another client's request for a different object was rejected.
        let unrelated = abort_payload(0x2000, 3, 0x0602_0000);
        assert!(matches!(
            client.process_response(&unrelated),
            SdoClientResult::IgnoredAbort
        ));

        // The active transfer still completes with the real response.
        let mut resp = [0u8; 8];
        resp[0] = (Scs::InitiateUploadResponse as u8) << 5 | 0x03; // expedited, size, n=0
        resp[4..8].copy_from_slice(&0xDEADBEEFu32.to_le_bytes());
        match client.process_response(&resp) {
            SdoClientResult::UploadComplete { data_len } => assert_eq!(data_len, 4),
            _ => panic!("expected UploadComplete"),
        }
    }

    #[test]
    fn client_cancels_on_abort_for_active_object() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let _req = client.start_upload(0x1000, 0);

        let matching = abort_payload(0x1000, 0, 0x0602_0000);
        match client.process_response(&matching) {
            SdoClientResult::Aborted(code) => assert_eq!(code, AbortCode::ObjectNotFound),
            _ => panic!("expected Aborted"),
        }
    }

    #[test]
    fn client_cancels_on_channel_level_abort() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let _req = client.start_upload(0x1000, 0);

        // Abort 0x0000:00 is channel-level and cancels the active transfer.
        let channel = abort_payload(0x0000, 0, 0x0504_0001);
        match client.process_response(&channel) {
            SdoClientResult::Aborted(code) => {
                assert_eq!(code, AbortCode::InvalidCommandSpecifier)
            }
            _ => panic!("expected Aborted"),
        }
    }

    #[test]
    fn client_abort_on_not_found() {
        let target = NodeId::new(1).unwrap();
        let mut client = SdoClient::<256>::new(target);
        let mut server = SdoServer::new();
        let mut od = TestOd {
            val_u32: 0,
            val_u16: 0,
            blob: [0; 20],
        };

        let req = client.start_upload(0xFFFF, 0);
        match run_transfer(&mut client, &mut server, &mut od, req) {
            SdoClientResult::Aborted(code) => {
                assert_eq!(code, AbortCode::ObjectNotFound);
            }
            _ => panic!("expected Aborted"),
        }
    }
}
