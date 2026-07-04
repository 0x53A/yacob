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
    total_len: usize, // expected size (0 if unknown)
    blksize: u8,
    seqno: u8, // last received sequence number within the current sub-block
    /// Set after the final segment (bit 7) is acknowledged; the next frame
    /// is expected to be the End Block Download request, not a segment.
    awaiting_end: bool,
    /// Client requested CRC verification in the initiate frame.
    crc_enabled: bool,
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
        // While receiving block download sub-blocks, byte 0 carries a sequence
        // number (1..=127, bit 7 set on the final segment), so it aliases every
        // command specifier. Per CiA 301 the only non-segment frame in this
        // phase is a client abort, which is exactly 0x80. Once the final
        // segment has been acknowledged (`awaiting_end`), normal dispatch
        // resumes so the End Block Download request (CCS=6) is routed below.
        if let Some(TransferState::BlockDownload(state)) = &self.transfer {
            if !state.awaiting_end && request[0] != 0x80 {
                return self.handle_block_download_segment(request, response, now_us);
            }
        }

        let cs = command_specifier(request[0]);

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
            let data_len = if size_indicated {
                4 - (((request[0] >> 2) & 0x03) as usize)
            } else {
                // Size not indicated: per CiA 301 the server uses the object's
                // known length; without it, all 4 data bytes are taken.
                od.lookup(index, subindex)
                    .and_then(|meta| meta.data_type.size())
                    .map_or(4, |size| size.min(4))
            };
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
                    crc: 0,
                    last_activity_us: now_us,
                }));

                Ok(())
            }
            3 => {
                // Start block upload (client says "go")
                match &mut self.transfer {
                    Some(TransferState::BlockUpload(state))
                        if state.seqno == 0 && state.offset == 0 =>
                    {
                        // Begin sending the first sub-block
                        state.seqno = 1;
                        state.last_activity_us = now_us;
                        Err(()) // no immediate response frame; poll_block_upload sends segments
                    }
                    _ => {
                        *response = encode_abort(0, 0, AbortCode::InvalidCommandSpecifier);
                        Ok(())
                    }
                }
            }
            2 => {
                // Sub-block ACK from client (cs=2, block upload response)
                match &mut self.transfer {
                    Some(TransferState::BlockUpload(state)) if state.seqno == 0 => {
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
                            let crc = state.crc;
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
                    }
                    Some(TransferState::BlockUpload(_)) => Err(()), // mid sub-block: ignore
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
                let crc_enabled = (request[0] & 0x04) != 0;

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
                    awaiting_end: false,
                    crc_enabled,
                    last_activity_us: now_us,
                }));

                Ok(())
            }
            1 => {
                // End block download
                match &mut self.transfer {
                    Some(TransferState::BlockDownload(state)) => {
                        let n_unused = ((request[0] >> 2) & 0x07) as usize;
                        let client_crc = (request[1] as u16) | ((request[2] as u16) << 8);

                        // The final segment always carries 7 data bytes, of
                        // which at least one must be valid, so only 0..=6 unused
                        // bytes are possible. Reject n=7 rather than silently
                        // truncating a full segment of payload.
                        if n_unused > 6 || state.offset < n_unused {
                            let (index, subindex) = (state.index, state.subindex);
                            self.transfer = None;
                            *response = encode_abort(index, subindex, AbortCode::DataTransferError);
                            return Ok(());
                        }

                        // Trim unused bytes from the last segment
                        state.offset -= n_unused;

                        let index = state.index;
                        let subindex = state.subindex;
                        let data_len = state.offset;
                        let total_len = state.total_len;
                        let crc_enabled = state.crc_enabled;
                        let mut write_buf = [0u8; 889];
                        write_buf[..data_len].copy_from_slice(&state.buf[..data_len]);
                        self.transfer = None;

                        // If the initiate request declared a size, the received
                        // data must match it (CiA 301).
                        if total_len != 0 && data_len != total_len {
                            *response = encode_abort(index, subindex, AbortCode::DataTransferError);
                            return Ok(());
                        }

                        // Verify the client's CRC over the received data
                        // (computed here rather than per-segment, since the
                        // final segment's padding is excluded).
                        if crc_enabled
                            && crc16_ccitt_update(0, &write_buf[..data_len]) != client_crc
                        {
                            *response = encode_abort(index, subindex, AbortCode::CrcError);
                            return Ok(());
                        }

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
        let expected_seqno = state.seqno.wrapping_add(1);
        if seqno == 0 || seqno != expected_seqno {
            let (index, subindex) = (state.index, state.subindex);
            self.transfer = None;
            *response = encode_abort(index, subindex, AbortCode::InvalidSequenceNumber);
            return Ok(());
        }

        // Copy 7 data bytes. A non-final segment that doesn't fit means the
        // transfer exceeds the buffer (possible when no size was indicated).
        let space = 889 - state.offset;
        if !is_last && space < 7 {
            let (index, subindex) = (state.index, state.subindex);
            self.transfer = None;
            *response = encode_abort(index, subindex, AbortCode::OutOfMemory);
            return Ok(());
        }
        let copy_len = space.min(7);
        state.buf[state.offset..state.offset + copy_len].copy_from_slice(&request[1..1 + copy_len]);
        state.offset += copy_len;
        state.seqno = seqno;
        state.last_activity_us = now_us;

        if is_last || seqno >= state.blksize {
            // End of sub-block — send ACK: SCS=5, ss=2 (block download response)
            *response = [0; 8];
            response[0] = (5 << 5) | 0x02;
            response[1] = state.seqno; // ackseq
            response[2] = state.blksize; // blksize for next sub-block
            if is_last {
                state.awaiting_end = true;
            } else {
                state.seqno = 0; // next sub-block restarts at seqno 1
            }
            Ok(())
        } else {
            Err(()) // no response for intermediate segments
        }
    }
}

/// CRC-16/CCITT with initial value 0 (aka CRC-16/XMODEM, polynomial 0x1021)
/// as specified by CiA 301 for block SDO transfers.
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
        big_data: [u8; 400],
        big_data_len: usize,
    }

    impl TestOd {
        fn new() -> Self {
            Self {
                device_type: 0x0000_0191,
                error_register: 0,
                writable_u16: 0x1234,
                long_data: [0xAA; 20],
                big_data: [0; 400],
                big_data_len: 0,
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
        OdEntryMeta {
            index: 0x2002,
            subindex: 0,
            data_type: DataType::OctetString,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "big_data",
            max_size: Some(400),
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
                (0x2002, 0) => {
                    buf[..self.big_data_len].copy_from_slice(&self.big_data[..self.big_data_len]);
                    Ok(self.big_data_len)
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
                (0x2002, 0) => {
                    if data.len() > 400 {
                        return Err(OdError::ValueTooLong);
                    }
                    self.big_data[..data.len()].copy_from_slice(data);
                    self.big_data_len = data.len();
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

    #[test]
    fn block_download_rejects_out_of_sequence_segment() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // Initiate block download to 0x2001:0 with a 7-byte size.
        let init_req = [0xC2, 0x01, 0x20, 0x00, 7, 0, 0, 0];
        server
            .process(
                &init_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();
        assert_eq!(resp[0], 0xA4);

        // First block segment must have seqno 1. Starting at 2 should abort.
        let bad_segment = [0x82, 1, 2, 3, 4, 5, 6, 7];
        server
            .process(
                &bad_segment,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                1,
            )
            .unwrap();

        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        assert_eq!(code, AbortCode::InvalidSequenceNumber as u32);
        // Abort must be addressed to the active transfer, not 0x0000:00.
        assert_eq!(u16::from_le_bytes([resp[1], resp[2]]), 0x2001);
        assert_eq!(resp[3], 0);
    }

    #[test]
    fn block_download_rejects_declared_size_mismatch() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // Declare an 8-byte block download to 0x2001:0.
        let init_req = [0xC2, 0x01, 0x20, 0x00, 8, 0, 0, 0];
        server
            .process(
                &init_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();
        assert_eq!(resp[0], 0xA4);

        // Send only 7 bytes and mark this as the final segment.
        let last_segment = [0x81, 1, 2, 3, 4, 5, 6, 7];
        server
            .process(
                &last_segment,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                1,
            )
            .unwrap();
        assert_eq!(resp[0], (5 << 5) | 0x02, "sub-block ACK");

        // End block download should abort because actual length != declared length.
        let end_req = [0xC1, 0, 0, 0, 0, 0, 0, 0];
        server
            .process(
                &end_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                2,
            )
            .unwrap();

        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        assert_eq!(code, AbortCode::DataTransferError as u32);
    }

    #[test]
    fn block_download_rejects_invalid_unused_byte_count() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // Initiate block download without size indication and without CRC,
        // so neither existing check can catch the truncation.
        let init_req = [0xC0, 0x01, 0x20, 0x00, 0, 0, 0, 0];
        server
            .process(
                &init_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();
        assert_eq!(resp[0], 0xA4);

        // One final 7-byte segment.
        let segment = [0x81, 1, 2, 3, 4, 5, 6, 7];
        server
            .process(
                &segment,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                1,
            )
            .unwrap();
        assert_eq!(resp[0], (5 << 5) | 0x02);

        // n_unused=7 is invalid for block download; only 0..=6 can be valid.
        let end_req = [0xC1 | (7 << 2), 0, 0, 0, 0, 0, 0, 0];
        server
            .process(
                &end_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                2,
            )
            .unwrap();

        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        // CiA 301 prescribes no abort code for an invalid n; this is our
        // implementation's choice, asserted to pin it down.
        assert_eq!(code, AbortCode::DataTransferError as u32);

        // The malformed end frame must not silently truncate and write data.
        assert_eq!(od.long_data, [0xAA; 20]);
    }

    #[test]
    fn block_download_roundtrip_completes() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // Initiate block download to 0x2001:0, 7 bytes, size indicated.
        let init_req = [0xC2, 0x01, 0x20, 0x00, 7, 0, 0, 0];
        server
            .process(
                &init_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();
        assert_eq!(resp[0], 0xA4);

        // Single final segment, seqno 1, 7 data bytes.
        let segment = [0x81, 1, 2, 3, 4, 5, 6, 7];
        server
            .process(
                &segment,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                1,
            )
            .unwrap();
        assert_eq!(resp[0], (5 << 5) | 0x02, "sub-block ACK expected");
        assert_eq!(resp[1], 1, "ackseq");

        // End block download (0xC1, n=0) must be routed to the end handler,
        // not misread as a data segment.
        let end_req = [0xC1, 0, 0, 0, 0, 0, 0, 0];
        server
            .process(
                &end_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                2,
            )
            .unwrap();
        assert_eq!(
            resp[0],
            (5 << 5) | 0x01,
            "end block ACK expected, got {:#04x} (abort code {:#010x})",
            resp[0],
            u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]])
        );
        assert_eq!(&od.long_data[..7], &[1, 2, 3, 4, 5, 6, 7]);
        // Transfer is finished.
        assert!(server.check_timeout(10_000_000).is_none());
    }

    #[test]
    fn block_download_with_more_than_31_segments_completes() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // 40 segments * 7 bytes = 280 bytes; seqno crosses 31 (where byte 0
        // no longer has command specifier 0) and 64/96 (aliasing other CCS
        // values), which previously misdispatched mid-block segments.
        let total: usize = 280;
        let mut payload = [0u8; 280];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }

        let init_req = [0xC2, 0x02, 0x20, 0x00, 24, 1, 0, 0]; // 280 LE
        server
            .process(
                &init_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();
        assert_eq!(resp[0], 0xA4);

        for seg in 0..40u8 {
            let seqno = seg + 1;
            let is_last = seqno == 40;
            let mut frame = [0u8; 8];
            frame[0] = seqno | if is_last { 0x80 } else { 0 };
            frame[1..8].copy_from_slice(&payload[seg as usize * 7..seg as usize * 7 + 7]);
            let result = server.process(
                &frame,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                seqno as u64,
            );
            if is_last {
                assert_eq!(result, Ok(()), "final segment must be ACKed");
                assert_eq!(
                    resp[0],
                    (5 << 5) | 0x02,
                    "sub-block ACK, got {:#04x}",
                    resp[0]
                );
                assert_eq!(resp[1], 40, "ackseq");
            } else {
                assert_eq!(
                    result,
                    Err(()),
                    "segment {seqno} must be consumed silently, got response {:#04x}",
                    resp[0]
                );
            }
        }

        let end_req = [0xC1, 0, 0, 0, 0, 0, 0, 0];
        server
            .process(
                &end_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                100,
            )
            .unwrap();
        assert_eq!(resp[0], (5 << 5) | 0x01, "end block ACK");
        assert_eq!(od.big_data_len, total);
        assert_eq!(&od.big_data[..total], &payload[..]);
    }

    #[test]
    fn block_download_abort_frame_cancels_transfer() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let init_req = [0xC2, 0x01, 0x20, 0x00, 14, 0, 0, 0];
        server
            .process(
                &init_req,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::PreOperational,
                0,
            )
            .unwrap();

        // Client abort (byte 0 == 0x80) mid-transfer: no response, state cleared.
        let abort = [0x80, 0x01, 0x20, 0x00, 0, 0, 4, 5];
        let result = server.process(
            &abort,
            &mut od,
            &mut resp,
            &mut events,
            NmtState::PreOperational,
            1,
        );
        assert_eq!(result, Err(()));
        assert!(
            server.check_timeout(10_000_000).is_none(),
            "transfer cleared"
        );
    }

    #[test]
    fn expedited_download_without_size_indication_uses_object_size() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // CCS=1, e=1, s=0 (0x22): expedited download, size not indicated.
        // All 4 data bytes are present, but 0x2000:0 is a u16 — the server
        // must clamp to the object size instead of failing the write.
        let req = [0x22, 0x00, 0x20, 0x00, 0xEF, 0xBE, 0xAD, 0xDE];
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
        assert_eq!(resp[0], 0x60, "download response, got {:#04x}", resp[0]);
        assert_eq!(od.writable_u16, 0xBEEF);
    }
}
