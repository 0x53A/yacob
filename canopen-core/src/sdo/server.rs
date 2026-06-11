use crate::nmt::NmtState;
use crate::od::{ObjectDictionary, OdError, OdEvent, OdEventSource};
use crate::sdo::protocol::*;
use heapless::Deque;

/// SDO server timeout for segmented transfers: 5 seconds in microseconds.
const SDO_TIMEOUT_US: u64 = 5_000_000;

/// Maximum block size (number of segments per sub-block). CiA 301 max is 127.
const MAX_BLKSIZE: u8 = 127;

/// State for a segmented transfer in progress.
struct SegmentedTransfer {
    index: u16,
    subindex: u8,
    toggle: bool,
    direction: Direction,
    buf: [u8; 889], // 127 * 7 = 889, enough for max block
    offset: usize,
    total_len: usize,
    last_activity_us: u64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Upload,   // reading from OD, sending to client
    Download, // receiving from client, writing to OD
}

/// State for an active block upload (server sending blocks to client).
struct BlockUploadState {
    index: u16,
    subindex: u8,
    buf: [u8; 889],
    total_len: usize,
    offset: usize, // how far we've sent
    blksize: u8,   // segments per sub-block
    seqno: u8,     // current segment number within sub-block (1-based)
    crc: u16,      // running CRC-16/CCITT
    last_activity_us: u64,
}

/// State for an active block download (server receiving blocks from client).
struct BlockDownloadState {
    index: u16,
    subindex: u8,
    buf: [u8; 889],
    offset: usize,
    #[allow(dead_code)] // retained for future size validation
    total_len: usize, // expected size (0 if unknown)
    blksize: u8,
    seqno: u8, // last received sequence number
    crc: u16,
    last_activity_us: u64,
}

enum TransferState {
    Segmented(SegmentedTransfer),
    BlockUpload(BlockUploadState),
    BlockDownload(BlockDownloadState),
}

impl TransferState {
    fn last_activity_us(&self) -> u64 {
        match self {
            Self::Segmented(t) => t.last_activity_us,
            Self::BlockUpload(t) => t.last_activity_us,
            Self::BlockDownload(t) => t.last_activity_us,
        }
    }

    fn index_sub(&self) -> (u16, u8) {
        match self {
            Self::Segmented(t) => (t.index, t.subindex),
            Self::BlockUpload(t) => (t.index, t.subindex),
            Self::BlockDownload(t) => (t.index, t.subindex),
        }
    }
}

/// SDO server state machine. Handles one transfer at a time.
pub struct SdoServer {
    transfer: Option<TransferState>,
}

impl SdoServer {
    pub const fn new() -> Self {
        Self { transfer: None }
    }

    /// Abort any in-progress transfer (e.g. on NMT reset).
    pub fn abort_transfer(&mut self) {
        self.transfer = None;
    }

    /// Check for timed-out transfers. Call periodically from Node::process().
    /// Returns an abort frame to send if a timeout occurred, or None.
    pub fn check_timeout(&mut self, now_us: u64) -> Option<[u8; 8]> {
        let timed_out = match &self.transfer {
            Some(t) => now_us.wrapping_sub(t.last_activity_us()) >= SDO_TIMEOUT_US,
            None => false,
        };
        if timed_out {
            let t = self.transfer.take().unwrap();
            let (idx, sub) = t.index_sub();
            Some(encode_abort(idx, sub, AbortCode::SdoProtocolTimeout))
        } else {
            None
        }
    }

    /// Poll for the next block upload segment to send. Call repeatedly from
    /// Node::process() until it returns None. Each call returns one CAN frame.
    pub fn poll_block_upload(&mut self, now_us: u64) -> Option<[u8; 8]> {
        let state = match &mut self.transfer {
            Some(TransferState::BlockUpload(s)) => s,
            _ => return None,
        };

        if state.seqno == 0 {
            // Waiting for client "start upload" or ACK — don't send
            return None;
        }

        let remaining = state.total_len - state.offset;
        let seg_data_len = remaining.min(7);
        let is_last_segment = remaining <= 7;
        let is_last_in_subblock = state.seqno >= state.blksize || is_last_segment;

        let mut frame = [0u8; 8];
        // Byte 0: bit 7 = last indicator, bits 0-6 = sequence number
        frame[0] = state.seqno;
        if is_last_segment && is_last_in_subblock {
            frame[0] |= 0x80; // last segment indicator
        }
        frame[1..1 + seg_data_len]
            .copy_from_slice(&state.buf[state.offset..state.offset + seg_data_len]);

        // Update CRC
        state.crc = crc16_ccitt_update(
            state.crc,
            &state.buf[state.offset..state.offset + seg_data_len],
        );

        state.offset += seg_data_len;
        state.last_activity_us = now_us;

        if is_last_in_subblock {
            state.seqno = 0; // wait for ACK
        } else {
            state.seqno += 1;
        }

        Some(frame)
    }

    /// Process an incoming SDO request frame and produce a response.
    ///
    /// `nmt_state` is used to enforce PDO config protection: writes to
    /// PDO communication/mapping parameters (0x1400-0x1BFF) are rejected
    /// when the node is in Operational state (per CiA 301).
    ///
    /// Returns `Ok(())` if a response was written to `response`.
    /// Returns `Err(())` if the frame was malformed and no response should be sent.
    pub fn process<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        nmt_state: NmtState,
        now_us: u64,
    ) -> Result<(), ()> {
        let cs = command_specifier(request[0]);

        // During active block download, CCS=0 frames are block segments
        if matches!(&self.transfer, Some(TransferState::BlockDownload(_))) && cs == 0 {
            return self.handle_block_download_segment(request, response, now_us);
        }

        match cs {
            cs if cs == Ccs::InitiateUpload as u8 => {
                self.handle_initiate_upload(request, od, response, now_us)
            }
            cs if cs == Ccs::UploadSegment as u8 => {
                self.handle_upload_segment(request, response, now_us)
            }
            cs if cs == Ccs::InitiateDownload as u8 => {
                self.handle_initiate_download(request, od, response, events, nmt_state, now_us)
            }
            cs if cs == Ccs::DownloadSegment as u8 => {
                self.handle_download_segment(request, od, response, events, nmt_state, now_us)
            }
            cs if cs == Ccs::BlockUpload as u8 => {
                self.handle_block_upload(request, od, response, now_us)
            }
            cs if cs == Ccs::BlockDownload as u8 => {
                self.handle_block_download(request, od, response, events, nmt_state, now_us)
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

    // ---- Segmented transfer handlers ----

    fn handle_initiate_upload<OD: ObjectDictionary>(
        &mut self,
        request: &[u8; 8],
        od: &OD,
        response: &mut [u8; 8],
        now_us: u64,
    ) -> Result<(), ()> {
        let (index, subindex) = parse_index_sub(request);

        let mut buf = [0u8; 889];
        let len = match od.read(index, subindex, &mut buf) {
            Ok(n) => n,
            Err(e) => {
                *response = encode_abort(index, subindex, od_error_to_abort(e));
                return Ok(());
            }
        };

        if len <= 4 {
            *response = encode_upload_response_expedited(index, subindex, &buf[..len]).unwrap();
            self.transfer = None;
        } else {
            *response = encode_upload_response_segmented(index, subindex, len as u32);
            self.transfer = Some(TransferState::Segmented(SegmentedTransfer {
                index,
                subindex,
                toggle: false,
                direction: Direction::Upload,
                buf,
                offset: 0,
                total_len: len,
                last_activity_us: now_us,
            }));
        }
        Ok(())
    }

    fn handle_upload_segment(
        &mut self,
        request: &[u8; 8],
        response: &mut [u8; 8],
        now_us: u64,
    ) -> Result<(), ()> {
        let transfer = match &mut self.transfer {
            Some(TransferState::Segmented(t)) if t.direction == Direction::Upload => t,
            _ => {
                *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                return Ok(());
            }
        };

        let client_toggle = (request[0] & 0x10) != 0;
        if client_toggle != transfer.toggle {
            let (idx, sub) = (transfer.index, transfer.subindex);
            self.transfer = None;
            *response = encode_abort(idx, sub, AbortCode::ToggleBitNotAlternated);
            return Ok(());
        }

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
        transfer.last_activity_us = now_us;

        if last {
            self.transfer = None;
        }
        Ok(())
    }

    fn handle_initiate_download<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        nmt_state: NmtState,
        now_us: u64,
    ) -> Result<(), ()> {
        let (index, subindex) = parse_index_sub(request);

        if nmt_state == NmtState::Operational && is_pdo_config_index(index) {
            *response = encode_abort(index, subindex, AbortCode::DataTransferDeviceState);
            return Ok(());
        }

        let expedited = (request[0] & 0x02) != 0;
        let size_indicated = (request[0] & 0x01) != 0;

        if expedited {
            let n = if size_indicated {
                ((request[0] >> 2) & 0x03) as usize
            } else {
                0
            };
            let data_len = 4 - n;
            let data = &request[4..4 + data_len];

            if let Err(e) = od.validate_write(index, subindex, data) {
                *response = encode_abort(index, subindex, od_error_to_abort(e));
                self.transfer = None;
                return Ok(());
            }

            match od.write(index, subindex, data) {
                Ok(()) => {
                    *response = encode_download_response(index, subindex);
                    push_event(
                        events,
                        OdEvent {
                            index,
                            subindex,
                            source: OdEventSource::Sdo,
                        },
                    );
                }
                Err(e) => {
                    *response = encode_abort(index, subindex, od_error_to_abort(e));
                }
            }
            self.transfer = None;
        } else {
            let total_len = if size_indicated {
                u32::from_le_bytes([request[4], request[5], request[6], request[7]]) as usize
            } else {
                0
            };

            if total_len > 889 {
                *response = encode_abort(index, subindex, AbortCode::OutOfMemory);
                return Ok(());
            }

            self.transfer = Some(TransferState::Segmented(SegmentedTransfer {
                index,
                subindex,
                toggle: false,
                direction: Direction::Download,
                buf: [0u8; 889],
                offset: 0,
                total_len,
                last_activity_us: now_us,
            }));
            *response = encode_download_response(index, subindex);
        }
        Ok(())
    }

    fn handle_download_segment<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        _nmt_state: NmtState,
        now_us: u64,
    ) -> Result<(), ()> {
        let transfer = match &mut self.transfer {
            Some(TransferState::Segmented(t)) if t.direction == Direction::Download => t,
            _ => {
                *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                return Ok(());
            }
        };

        let client_toggle = (request[0] & 0x10) != 0;
        if client_toggle != transfer.toggle {
            let (idx, sub) = (transfer.index, transfer.subindex);
            self.transfer = None;
            *response = encode_abort(idx, sub, AbortCode::ToggleBitNotAlternated);
            return Ok(());
        }

        let last = (request[0] & 0x01) != 0;
        let n = ((request[0] >> 1) & 0x07) as usize;
        let seg_len = 7 - n;

        if transfer.offset + seg_len > 889 {
            let (idx, sub) = (transfer.index, transfer.subindex);
            self.transfer = None;
            *response = encode_abort(idx, sub, AbortCode::OutOfMemory);
            return Ok(());
        }

        transfer.buf[transfer.offset..transfer.offset + seg_len]
            .copy_from_slice(&request[1..1 + seg_len]);
        transfer.offset += seg_len;
        transfer.toggle = !transfer.toggle;
        transfer.last_activity_us = now_us;

        *response = encode_download_segment_response(client_toggle);

        if last {
            let index = transfer.index;
            let subindex = transfer.subindex;
            let data_len = transfer.offset;
            // Copy out the data we need before clearing transfer
            let mut write_buf = [0u8; 889];
            write_buf[..data_len].copy_from_slice(&transfer.buf[..data_len]);
            self.transfer = None;

            if let Err(e) = od.validate_write(index, subindex, &write_buf[..data_len]) {
                *response = encode_abort(index, subindex, od_error_to_abort(e));
                return Ok(());
            }

            match od.write(index, subindex, &write_buf[..data_len]) {
                Ok(()) => {
                    push_event(
                        events,
                        OdEvent {
                            index,
                            subindex,
                            source: OdEventSource::Sdo,
                        },
                    );
                }
                Err(e) => {
                    *response = encode_abort(index, subindex, od_error_to_abort(e));
                }
            }
        }

        Ok(())
    }

    // ---- Block transfer handlers ----

    fn handle_block_upload<OD: ObjectDictionary>(
        &mut self,
        request: &[u8; 8],
        od: &OD,
        response: &mut [u8; 8],
        now_us: u64,
    ) -> Result<(), ()> {
        // CCS=5: sub-command is in bits 0-1 of byte 0
        let sub_cmd = request[0] & 0x03;

        match sub_cmd {
            0 => {
                // Initiate block upload
                let (index, subindex) = parse_index_sub(request);
                let client_blksize = request[4];
                let _crc_supported = (request[0] & 0x04) != 0;

                let mut buf = [0u8; 889];
                let len = match od.read(index, subindex, &mut buf) {
                    Ok(n) => n,
                    Err(e) => {
                        *response = encode_abort(index, subindex, od_error_to_abort(e));
                        return Ok(());
                    }
                };

                let blksize = client_blksize.min(MAX_BLKSIZE);

                // SCS=6, sc=0 (initiate), ss=1 (size indicated), cc=1 (CRC supported)
                *response = [0; 8];
                response[0] = (6 << 5) | 0x04 | 0x02; // SCS=6, CRC supported, size indicated
                response[1] = (index & 0xFF) as u8;
                response[2] = (index >> 8) as u8;
                response[3] = subindex;
                response[4..8].copy_from_slice(&(len as u32).to_le_bytes());

                self.transfer = Some(TransferState::BlockUpload(BlockUploadState {
                    index,
                    subindex,
                    buf,
                    total_len: len,
                    offset: 0,
                    blksize,
                    seqno: 0, // waiting for "start upload" from client
                    crc: 0xFFFF,
                    last_activity_us: now_us,
                }));

                Ok(())
            }
            3 => {
                // Start block upload (client says "go") or ACK sub-block
                match &mut self.transfer {
                    Some(TransferState::BlockUpload(state)) => {
                        if state.seqno == 0 && state.offset == 0 {
                            // "Start upload" — begin sending first sub-block
                            state.seqno = 1;
                            state.last_activity_us = now_us;
                            Err(()) // no immediate response frame; poll_block_upload sends segments
                        } else if state.seqno == 0 {
                            // ACK for a sub-block
                            let _ackseq = request[1];
                            let new_blksize = request[2];
                            state.blksize = new_blksize.min(MAX_BLKSIZE);
                            state.last_activity_us = now_us;

                            if state.offset >= state.total_len {
                                // All data sent — send end block upload
                                let n_unused = if state.total_len % 7 == 0 {
                                    0
                                } else {
                                    7 - (state.total_len % 7)
                                };
                                let crc = finalize_crc16(state.crc);
                                // SCS=6, sc=1 (end), n in bits 2-4
                                *response = [0; 8];
                                response[0] = (6 << 5) | 0x01 | ((n_unused as u8 & 0x07) << 2);
                                response[1] = (crc & 0xFF) as u8;
                                response[2] = (crc >> 8) as u8;
                                self.transfer = None;
                                Ok(())
                            } else {
                                // More data to send — start next sub-block
                                state.seqno = 1;
                                Err(()) // poll_block_upload sends segments
                            }
                        } else {
                            Err(()) // unexpected
                        }
                    }
                    _ => {
                        *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                        Ok(())
                    }
                }
            }
            1 => {
                // End block upload confirmation from client
                self.transfer = None;
                Err(()) // no response needed
            }
            _ => {
                *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                Ok(())
            }
        }
    }

    fn handle_block_download<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        nmt_state: NmtState,
        now_us: u64,
    ) -> Result<(), ()> {
        let sub_cmd = request[0] & 0x01;

        match sub_cmd {
            0 => {
                // Initiate block download
                let (index, subindex) = parse_index_sub(request);
                let size_indicated = (request[0] & 0x02) != 0;
                let _crc_supported = (request[0] & 0x04) != 0;

                if nmt_state == NmtState::Operational && is_pdo_config_index(index) {
                    *response = encode_abort(index, subindex, AbortCode::DataTransferDeviceState);
                    return Ok(());
                }

                let total_len = if size_indicated {
                    u32::from_le_bytes([request[4], request[5], request[6], request[7]]) as usize
                } else {
                    0
                };

                if total_len > 889 {
                    *response = encode_abort(index, subindex, AbortCode::OutOfMemory);
                    return Ok(());
                }

                // SCS=5, sc=0, ss=0, CRC supported
                *response = [0; 8];
                response[0] = (5 << 5) | 0x04; // SCS=5, CRC supported
                response[1] = (index & 0xFF) as u8;
                response[2] = (index >> 8) as u8;
                response[3] = subindex;
                response[4] = MAX_BLKSIZE;

                self.transfer = Some(TransferState::BlockDownload(BlockDownloadState {
                    index,
                    subindex,
                    buf: [0u8; 889],
                    offset: 0,
                    total_len,
                    blksize: MAX_BLKSIZE,
                    seqno: 0,
                    crc: 0xFFFF,
                    last_activity_us: now_us,
                }));

                Ok(())
            }
            1 => {
                // End block download
                match &mut self.transfer {
                    Some(TransferState::BlockDownload(state)) => {
                        let n_unused = ((request[0] >> 2) & 0x07) as usize;
                        let _client_crc = (request[1] as u16) | ((request[2] as u16) << 8);

                        // Trim unused bytes from the last segment
                        if state.offset >= n_unused {
                            state.offset -= n_unused;
                        }

                        let index = state.index;
                        let subindex = state.subindex;
                        let data_len = state.offset;
                        let mut write_buf = [0u8; 889];
                        write_buf[..data_len].copy_from_slice(&state.buf[..data_len]);
                        self.transfer = None;

                        // SCS=5, sc=1 (end)
                        *response = [0; 8];
                        response[0] = (5 << 5) | 0x01;

                        if let Err(e) = od.validate_write(index, subindex, &write_buf[..data_len]) {
                            *response = encode_abort(index, subindex, od_error_to_abort(e));
                            return Ok(());
                        }

                        match od.write(index, subindex, &write_buf[..data_len]) {
                            Ok(()) => {
                                push_event(
                                    events,
                                    OdEvent {
                                        index,
                                        subindex,
                                        source: OdEventSource::Sdo,
                                    },
                                );
                            }
                            Err(e) => {
                                *response = encode_abort(index, subindex, od_error_to_abort(e));
                            }
                        }

                        Ok(())
                    }
                    _ => {
                        *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                        Ok(())
                    }
                }
            }
            _ => {
                *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                Ok(())
            }
        }
    }

    /// Process a block download segment (CCS=0 during active block download).
    /// Called from process() when a block download is in progress and the
    /// incoming frame is a data segment (not a normal download segment).
    fn handle_block_download_segment(
        &mut self,
        request: &[u8; 8],
        response: &mut [u8; 8],
        now_us: u64,
    ) -> Result<(), ()> {
        let state = match &mut self.transfer {
            Some(TransferState::BlockDownload(s)) => s,
            _ => {
                *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                return Ok(());
            }
        };

        let seqno = request[0] & 0x7F;
        let is_last = (request[0] & 0x80) != 0;

        // Copy 7 data bytes
        let space = 889 - state.offset;
        let copy_len = space.min(7);
        state.buf[state.offset..state.offset + copy_len].copy_from_slice(&request[1..1 + copy_len]);
        state.crc = crc16_ccitt_update(state.crc, &request[1..1 + copy_len]);
        state.offset += copy_len;
        state.seqno = seqno;
        state.last_activity_us = now_us;

        if is_last || seqno >= state.blksize {
            // End of sub-block — send ACK
            // SCS=2 (block download response)
            *response = [0; 8];
            response[0] = 2 << 5; // SCS=2
            response[1] = state.seqno; // ackseq
            response[2] = state.blksize; // blksize for next sub-block
            Ok(())
        } else {
            Err(()) // no response for intermediate segments
        }
    }
}

/// CRC-16/CCITT (polynomial 0x1021, init 0xFFFF) — used by block SDO.
fn crc16_ccitt_update(crc: u16, data: &[u8]) -> u16 {
    let mut crc = crc;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

fn finalize_crc16(crc: u16) -> u16 {
    crc
}

/// Push an event to the queue, dropping the oldest if full.
fn push_event<const N: usize>(events: &mut Deque<OdEvent, N>, event: OdEvent) {
    if events.is_full() {
        let _ = events.pop_front();
    }
    let _ = events.push_back(event);
}

/// Returns true if the index is a PDO communication or mapping parameter (0x1400-0x1BFF).
fn is_pdo_config_index(index: u16) -> bool {
    index >= 0x1400 && index <= 0x1BFF
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
    use crate::datatypes::DataType;
    use crate::od::*;

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
            index: 0x1000,
            subindex: 0,
            data_type: DataType::U32,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "device_type",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1001,
            subindex: 0,
            data_type: DataType::U8,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "error_register",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x2000,
            subindex: 0,
            data_type: DataType::U16,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "writable_u16",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x2001,
            subindex: 0,
            data_type: DataType::OctetString,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "long_data",
            max_size: None,
        },
    ];

    impl ObjectDictionary for TestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            TEST_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
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
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();

        // Should be expedited response with 4 bytes
        assert_eq!(
            command_specifier(resp[0]),
            Scs::InitiateUploadResponse as u8
        );
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
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();

        assert_eq!(
            command_specifier(resp[0]),
            Scs::InitiateDownloadResponse as u8
        );
        assert_eq!(od.writable_u16, 0xABCD);

        // Should have generated an event
        let evt = events.pop_front().unwrap();
        assert_eq!(evt.index, 0x2000);
        assert_eq!(evt.subindex, 0);
        assert_eq!(evt.source, OdEventSource::Sdo);
    }

    #[test]
    fn upload_read_only_write_rejected() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Try to write to 0x1000:0 (read-only)
        let req = [0x23, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00];
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();

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
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();

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
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();

        // Should be segmented initiate response
        assert_eq!(
            command_specifier(resp[0]),
            Scs::InitiateUploadResponse as u8
        );
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
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ];
            server
                .process(
                    &seg_req,
                    &mut od,
                    &mut resp,
                    &mut events,
                    NmtState::Operational,
                    0,
                )
                .unwrap();

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

    #[test]
    fn pdo_config_write_rejected_in_operational() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Try to write to 0x1800:01 (TPDO comm param) while Operational
        let req = [0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0x00];
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();

        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        assert_eq!(code, AbortCode::DataTransferDeviceState as u32);
    }

    #[test]
    fn pdo_config_write_allowed_in_preoperational() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Same write to 0x1800:01 but in PreOperational — should not abort
        // (will get ObjectNotFound since TestOd doesn't have 0x1800, but that's OK —
        // the point is it's not rejected by the PDO config guard)
        let req = [0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0x00];
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();

        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        // Should be ObjectNotFound (OD doesn't have 0x1800), NOT DataTransferDeviceState
        assert_eq!(code, AbortCode::ObjectNotFound as u32);
    }

    #[test]
    fn sdo_timeout_aborts_stale_transfer() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];

        // Initiate segmented upload for 20-byte entry
        let req = [0x40, 0x01, 0x20, 0x00, 0, 0, 0, 0];
        let mut events: Deque<OdEvent, 16> = Deque::new();
        server
            .process(
                &req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                1_000_000,
            )
            .unwrap();

        // Transfer is now in progress. Check timeout before 5s — should be None
        assert!(server.check_timeout(3_000_000).is_none());

        // Check timeout after 5s — should abort
        let abort = server.check_timeout(7_000_000).unwrap();
        assert_eq!(command_specifier(abort[0]), Scs::AbortTransfer as u8);
        let code = u32::from_le_bytes([abort[4], abort[5], abort[6], abort[7]]);
        assert_eq!(code, AbortCode::SdoProtocolTimeout as u32);

        // Transfer should be cleared
        assert!(server.check_timeout(10_000_000).is_none());
    }
}
