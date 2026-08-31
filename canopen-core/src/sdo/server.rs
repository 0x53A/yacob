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
    /// Offset at which the current sub-block started. Used with the client's
    /// ACK sequence number to rewind to the first unacknowledged segment.
    subblock_start_offset: usize,
    blksize: u8, // segments per sub-block
    seqno: u8,   // current segment number within sub-block (1-based)
    /// Number of the last segment offered in the current sub-block. `seqno`
    /// becomes zero while waiting for the ACK, so this is retained separately.
    last_seqno: u8,
    crc: u16, // running CRC-16/CCITT
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

/// The COB-IDs this node's SDO servers occupy — the default 0x1200 channel
/// plus every additional 0x1201+ channel, already resolved against the node id.
///
/// Config downloads that would move another service onto one of these are
/// rejected: [`Node::dispatch_frame`](crate::node::Node) matches SDO channels
/// first, so a colliding PDO would be silently swallowed by an SDO server.
/// `Node::new` enforces the same invariant for the static config; this carries
/// it into the runtime, where CiA 301 lets a client rewrite PDO COB-IDs and
/// heartbeat consumer entries in Pre-Operational.
///
/// The default (all zero, no extras) claims nothing — a standalone `SdoServer`
/// with no node around it has no COB-ID plan to protect.
#[derive(Clone, Copy, Default)]
pub struct SdoCobIds<'a> {
    /// Default server request COB-ID (`0x600 + node_id`).
    pub default_rx: u16,
    /// Default server response COB-ID (`0x580 + node_id`).
    pub default_tx: u16,
    /// Request COB-IDs of the additional servers (0x1201+).
    pub extra_rx: &'a [u16],
    /// Response COB-IDs of the additional servers (0x1201+).
    pub extra_tx: &'a [u16],
}

impl SdoCobIds<'_> {
    /// True if an SDO server listens on or answers with `cob`. COB-ID 0 is
    /// never claimed — it is the "unresolved" placeholder, not a real ID.
    pub fn claims(&self, cob: u16) -> bool {
        cob != 0
            && (cob == self.default_rx
                || cob == self.default_tx
                || self.extra_rx.contains(&cob)
                || self.extra_tx.contains(&cob))
    }
}

/// COB-IDs services *other than* SDO occupy on the owning node: every declared
/// PDO (parked ones included — a disabled PDO can be enabled later without its
/// COB-ID being rewritten), EMCY, this node's own heartbeat, the heartbeat
/// producers it monitors, and LSS.
///
/// Used to reject a config download that would move the SYNC object onto one
/// of them. `Node::dispatch_frame` matches SYNC ahead of all of these and
/// returns unconditionally, so such a relocation silently mutes the other
/// service — the same failure the SDO-channel checks exist to prevent.
///
/// The default (empty, not truncated) claims nothing: a standalone
/// [`SdoServer`] has no node around it and no COB-ID plan to protect.
#[derive(Clone, Copy, Default)]
pub struct ReservedCobIds<'a> {
    /// The occupied COB-IDs. 0 is never claimed — it is the "unresolved"
    /// placeholder, not a real ID.
    pub ids: &'a [u16],
    /// Set when the node uses more COB-IDs than the list could hold. The
    /// remaining ones are unknown here, so a relocation that cannot be proved
    /// safe is rejected rather than allowed through unchecked.
    pub truncated: bool,
}

impl ReservedCobIds<'_> {
    /// True if another service occupies `cob`, or if the list is truncated and
    /// therefore cannot rule it out.
    pub fn claims(&self, cob: u16) -> bool {
        cob != 0 && (self.truncated || self.ids.contains(&cob))
    }
}

/// Per-request context for [`SdoServer::process_with_context`]: everything the
/// server needs beyond the request frame and the OD itself.
#[derive(Clone, Copy)]
pub struct SdoContext<'a> {
    /// PDO communication/mapping parameters (0x1400-0x1BFF) are locked while
    /// Operational (CiA 301).
    pub nmt_state: NmtState,
    /// Current time in microseconds, for segmented/block transfer timeouts.
    pub now_us: u64,
    /// Number of 0x1016 consumer-heartbeat subindices the owning node actively
    /// monitors. Entries above this remain writable but cannot collide with a
    /// monitor that does not exist.
    pub heartbeat_consumer_count: u8,
    /// COB-IDs the node's SDO servers occupy; config downloads may not steal
    /// them. Default (nothing claimed) skips the check.
    pub cob_ids: SdoCobIds<'a>,
    /// COB-IDs the node's non-SDO services occupy; a 0x1005 download may not
    /// move SYNC onto them. See [`ReservedCobIds`].
    pub reserved_cob_ids: ReservedCobIds<'a>,
    /// Current SYNC COB-ID, or 0 when the node has no usable SYNC object.
    /// PDO and monitored-heartbeat downloads may not move onto it.
    pub sync_cob_id: u16,
}

/// SDO server state machine. Handles one transfer at a time.
pub struct SdoServer {
    transfer: Option<TransferState>,
    /// (index, subindex) of the most recent committed download. Taken by the
    /// node to resync runtime config (PDO engines, heartbeat) — unlike the OD
    /// event queue, this signal cannot be lost to queue overflow.
    committed_write: Option<(u16, u8)>,
}

impl SdoServer {
    pub const fn new() -> Self {
        Self {
            transfer: None,
            committed_write: None,
        }
    }

    /// (index, subindex) of the download committed by the last `process()`
    /// call, if any. Clears the signal; each SDO request commits at most one
    /// write, so take this after every `process()`.
    pub fn take_committed_write(&mut self) -> Option<(u16, u8)> {
        self.committed_write.take()
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
        state.last_seqno = frame[0] & 0x7F;

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
    ///
    /// Downloads are validated against the OD only. Use
    /// [`process_with_context`](Self::process_with_context) to also reject
    /// config writes that would park another service on an SDO channel;
    /// [`Node`](crate::node::Node) does that for its servers.
    pub fn process<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        nmt_state: NmtState,
        now_us: u64,
    ) -> Result<(), ()> {
        let ctx = SdoContext {
            nmt_state,
            now_us,
            heartbeat_consumer_count: 0,
            cob_ids: SdoCobIds::default(),
            reserved_cob_ids: ReservedCobIds::default(),
            sync_cob_id: 0,
        };
        self.process_with_context(request, od, response, events, ctx)
    }

    /// [`process`](Self::process) with the owning node's COB-ID plan, so
    /// configuration downloads cannot create collisions involving SYNC or an
    /// SDO channel. A collision aborts with 0x0604_0043 instead of silently
    /// creating a frame the dispatcher misroutes. See [`SdoCobIds`] and
    /// [`ReservedCobIds`].
    pub fn process_with_context<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        request: &[u8; 8],
        od: &mut OD,
        response: &mut [u8; 8],
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        ctx: SdoContext<'_>,
    ) -> Result<(), ()> {
        let now_us = ctx.now_us;
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
                self.handle_initiate_download(request, od, response, events, ctx)
            }
            cs if cs == Ccs::DownloadSegment as u8 => {
                self.handle_download_segment(request, od, response, events, ctx)
            }
            cs if cs == Ccs::BlockUpload as u8 => {
                self.handle_block_upload(request, od, response, now_us)
            }
            cs if cs == Ccs::BlockDownload as u8 => {
                self.handle_block_download(request, od, response, events, ctx)
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
                *response = encode_sdo_abort(index, subindex, od_error_to_abort(e));
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
        ctx: SdoContext<'_>,
    ) -> Result<(), ()> {
        let validation_ctx = ctx;
        let SdoContext {
            nmt_state, now_us, ..
        } = ctx;
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

            if let Err(code) = validate_download(od, validation_ctx, index, subindex, data) {
                *response = encode_sdo_abort(index, subindex, code);
                self.transfer = None;
                return Ok(());
            }

            match od.write(index, subindex, data) {
                Ok(()) => {
                    *response = encode_download_response(index, subindex);
                    self.committed_write = Some((index, subindex));
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
                    *response = encode_sdo_abort(index, subindex, od_error_to_abort(e));
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
        ctx: SdoContext<'_>,
    ) -> Result<(), ()> {
        let validation_ctx = ctx;
        let now_us = ctx.now_us;
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

            if let Err(code) =
                validate_download(od, validation_ctx, index, subindex, &write_buf[..data_len])
            {
                *response = encode_sdo_abort(index, subindex, code);
                return Ok(());
            }

            match od.write(index, subindex, &write_buf[..data_len]) {
                Ok(()) => {
                    self.committed_write = Some((index, subindex));
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
                    *response = encode_sdo_abort(index, subindex, od_error_to_abort(e));
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

                if !(1..=MAX_BLKSIZE).contains(&client_blksize) {
                    *response = encode_abort(index, subindex, AbortCode::InvalidBlockSize);
                    return Ok(());
                }

                let mut buf = [0u8; 889];
                let len = match od.read(index, subindex, &mut buf) {
                    Ok(n) => n,
                    Err(e) => {
                        *response = encode_sdo_abort(index, subindex, od_error_to_abort(e));
                        return Ok(());
                    }
                };

                let blksize = client_blksize;

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
                    subblock_start_offset: 0,
                    blksize,
                    seqno: 0, // waiting for "start upload" from client
                    last_seqno: 0,
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
                        let ackseq = request[1];
                        let new_blksize = request[2];
                        if !(1..=MAX_BLKSIZE).contains(&new_blksize) {
                            let (index, subindex) = (state.index, state.subindex);
                            self.transfer = None;
                            *response = encode_abort(index, subindex, AbortCode::InvalidBlockSize);
                            return Ok(());
                        }
                        if ackseq > state.last_seqno {
                            let (index, subindex) = (state.index, state.subindex);
                            self.transfer = None;
                            *response =
                                encode_abort(index, subindex, AbortCode::InvalidSequenceNumber);
                            return Ok(());
                        }

                        // ACK may stop before the end of the sub-block. Resume
                        // with the first unacknowledged payload byte, numbering
                        // the retransmission sub-block from one as required by
                        // the block protocol.
                        let acknowledged = usize::from(ackseq) * 7;
                        state.offset =
                            (state.subblock_start_offset + acknowledged).min(state.total_len);
                        state.crc = crc16_ccitt_update(0, &state.buf[..state.offset]);
                        state.subblock_start_offset = state.offset;
                        state.blksize = new_blksize;
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
                            state.last_seqno = 0;
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
        ctx: SdoContext<'_>,
    ) -> Result<(), ()> {
        let validation_ctx = ctx;
        let SdoContext {
            nmt_state, now_us, ..
        } = ctx;
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

                        if let Err(code) = validate_download(
                            od,
                            validation_ctx,
                            index,
                            subindex,
                            &write_buf[..data_len],
                        ) {
                            *response = encode_sdo_abort(index, subindex, code);
                            return Ok(());
                        }

                        match od.write(index, subindex, &write_buf[..data_len]) {
                            Ok(()) => {
                                self.committed_write = Some((index, subindex));
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
                                *response = encode_sdo_abort(index, subindex, od_error_to_abort(e));
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

/// Validate a download before committing it: CiA 301 semantic rules for
/// communication-profile objects that plain OD type checking cannot express,
/// then the OD's application-level `validate_write` hook.
fn validate_download<OD: ObjectDictionary>(
    od: &OD,
    ctx: SdoContext<'_>,
    index: u16,
    subindex: u8,
    data: &[u8],
) -> Result<(), SdoAbort> {
    validate_heartbeat_consumer_write(
        od,
        ctx.cob_ids,
        ctx.sync_cob_id,
        ctx.heartbeat_consumer_count,
        index,
        subindex,
        data,
    )?;
    validate_pdo_cob_id_write(ctx.cob_ids, ctx.sync_cob_id, index, subindex, data)?;
    validate_sync_cob_id_write(ctx.cob_ids, ctx.reserved_cob_ids, index, subindex, data)?;
    validate_sync_counter_overflow_write(index, subindex, data)?;
    od.validate_write(index, subindex, data)
        .map_err(od_error_to_abort)
}

/// CiA 301 rules for 0x1005 (COB-ID SYNC message).
///
/// Malformed values abort with 0x0609_0030 rather than being accepted and
/// masked away, so a master learns the SYNC object did not move where it
/// asked: bit 29 selects 29-bit SYNC frames (not implemented here), COB-ID 0
/// is the NMT COB-ID and never a SYNC object, and the reserved bits (31 and
/// 11..=28 — 0x1005 has no valid/invalid bit, unlike a PDO COB-ID) must be
/// zero.
///
/// SYNC must also not be parked on a COB-ID another service on this node
/// already occupies. `Node::dispatch_frame` matches SYNC ahead of everything
/// but the SDO channels and returns unconditionally, so a collision mutes the
/// other service outright: an RPDO would stop being received, a monitored
/// heartbeat would look like a timeout, LSS would go deaf. Rejected with
/// 0x0604_0043, as for a PDO parked on an SDO channel
/// (see [`validate_pdo_cob_id_write`]).
fn validate_sync_cob_id_write(
    sdo_cobs: SdoCobIds<'_>,
    reserved_cobs: ReservedCobIds<'_>,
    index: u16,
    subindex: u8,
    data: &[u8],
) -> Result<(), AbortCode> {
    if index != 0x1005 || subindex != 0 || data.len() != 4 {
        return Ok(());
    }
    let raw = u32::from_le_bytes(data.try_into().unwrap());
    if !crate::sync::is_supported_sync_cob_id(raw) {
        return Err(AbortCode::ValueRangeExceeded);
    }
    let cob = (raw & 0x7FF) as u16;
    if sdo_cobs.claims(cob) || reserved_cobs.claims(cob) {
        return Err(AbortCode::ParameterIncompatibility);
    }
    Ok(())
}

/// CiA 301 rules for 0x1019 (synchronous counter overflow value): 0 disables
/// the SYNC counter, and 2..=240 selects the value it wraps at. 1 is not a
/// legal value, and neither is anything above 240 — accepting either would
/// leave the producer emitting counters the spec has no meaning for, or
/// silently treating the entry as disabled.
fn validate_sync_counter_overflow_write(
    index: u16,
    subindex: u8,
    data: &[u8],
) -> Result<(), AbortCode> {
    if index != 0x1019 || subindex != 0 || data.is_empty() {
        return Ok(());
    }
    if !crate::sync::is_valid_sync_counter_overflow(data[0]) {
        return Err(AbortCode::ValueRangeExceeded);
    }
    Ok(())
}

/// A PDO must not be parked on a COB-ID an SDO server or the SYNC object
/// occupies: those services dispatch first, so such a PDO would never reach
/// the PDO engine (and an SDO request could be answered on the PDO's COB-ID).
/// Rejected with 0x0604_0043, as for the duplicate heartbeat producer.
///
/// The valid bit (31) is deliberately ignored: a disabled PDO can be enabled
/// later by a write that carries no COB-ID, so the collision has to be caught
/// when the COB-ID is written, not when it goes live. Only the 11-bit ID is
/// compared, matching `Node::sync_pdo_from_od` — the stack has no extended-ID
/// PDOs, so the frame-type bit does not make a colliding ID distinct.
fn validate_pdo_cob_id_write(
    sdo_cobs: SdoCobIds<'_>,
    sync_cob_id: u16,
    index: u16,
    subindex: u8,
    data: &[u8],
) -> Result<(), AbortCode> {
    let is_comm_param = (0x1400..=0x15FF).contains(&index) || (0x1800..=0x19FF).contains(&index);
    if !is_comm_param || subindex != 1 || data.len() != 4 {
        return Ok(());
    }
    let raw = u32::from_le_bytes(data.try_into().unwrap());
    let cob = (raw & 0x7FF) as u16;
    if sdo_cobs.claims(cob) || (cob != 0 && cob == sync_cob_id) {
        return Err(AbortCode::ParameterIncompatibility);
    }
    Ok(())
}

/// CiA 301 rules for 0x1016 (Consumer heartbeat time) entries: reserved bits
/// 24..32 must always be zero; a non-disabled entry (consumer time != 0) must
/// additionally carry a valid producer node id (1..=127), and no two entries
/// may monitor the same node (abort 0x0604_0043). The producer's heartbeat
/// COB-ID must also not be one an SDO server occupies, for the same reason a
/// PDO may not be parked there (see [`validate_pdo_cob_id_write`]). The same
/// rule protects the current SYNC COB-ID: configuration order must not decide
/// whether a collision is accepted.
fn validate_heartbeat_consumer_write<OD: ObjectDictionary>(
    od: &OD,
    sdo_cobs: SdoCobIds<'_>,
    sync_cob_id: u16,
    heartbeat_consumer_count: u8,
    index: u16,
    subindex: u8,
    data: &[u8],
) -> Result<(), AbortCode> {
    if index != 0x1016 || subindex == 0 || data.len() != 4 {
        return Ok(());
    }
    let raw = u32::from_le_bytes(data.try_into().unwrap());
    if raw >> 24 != 0 {
        return Err(AbortCode::ValueRangeExceeded);
    }
    if raw & 0xFFFF == 0 {
        // Consumer time 0 disables the entry — node id bits don't matter.
        return Ok(());
    }
    let node = (raw >> 16) & 0xFF;
    if node == 0 || node > 127 {
        return Err(AbortCode::ValueRangeExceeded);
    }

    if subindex <= heartbeat_consumer_count {
        let cob = 0x700 + node as u16;
        if sdo_cobs.claims(cob) || cob == sync_cob_id {
            return Err(AbortCode::ParameterIncompatibility);
        }
    }
    // The same producer must not be monitored by two active entries.
    let count = od.sub_count(index).unwrap_or(0);
    let mut buf = [0u8; 4];
    for sub in 1..=count {
        if sub == subindex {
            continue;
        }
        if od.read(index, sub, &mut buf).is_ok() {
            let other = u32::from_le_bytes(buf);
            if other & 0xFFFF != 0 && (other >> 16) & 0xFF == node {
                return Err(AbortCode::ParameterIncompatibility);
            }
        }
    }
    Ok(())
}

/// Returns true if the index is a PDO communication or mapping parameter (0x1400-0x1BFF).
fn is_pdo_config_index(index: u16) -> bool {
    index >= 0x1400 && index <= 0x1BFF
}

fn od_error_to_abort(e: OdError) -> SdoAbort {
    match e {
        OdError::NotFound => AbortCode::ObjectNotFound.into(),
        OdError::ReadOnly => AbortCode::ReadOnlyObject.into(),
        OdError::WriteOnly => AbortCode::WriteOnlyObject.into(),
        OdError::DataTypeMismatch => AbortCode::DataTypeMismatch.into(),
        OdError::ValueTooLong => AbortCode::OutOfMemory.into(),
        OdError::ValueRange => AbortCode::ValueRangeExceeded.into(),
        OdError::HardwareError => AbortCode::GeneralError.into(),
        OdError::Abort(code) => code,
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
    fn block_upload_retransmits_after_partial_ack() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        for (i, byte) in od.long_data.iter_mut().enumerate() {
            *byte = i as u8;
        }
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // Initiate a CRC-capable block upload with three segments per block.
        let init = [0xA4, 0x01, 0x20, 0x00, 3, 0, 0, 0];
        server
            .process(
                &init,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();
        assert_eq!(resp[0], 0xC6);

        let start = [0xA3, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(
            server.process(
                &start,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                1,
            ),
            Err(())
        );

        let first = server.poll_block_upload(2).unwrap();
        let second = server.poll_block_upload(3).unwrap();
        let third = server.poll_block_upload(4).unwrap();
        assert_eq!(first[0], 1);
        assert_eq!(second[0], 2);
        assert_eq!(third[0], 0x83, "final segment in the first attempt");

        // Only segment 1 arrived. The next sub-block must restart at byte 7,
        // not skip ahead to the end of the object.
        let partial_ack = [0xA2, 1, 2, 0, 0, 0, 0, 0];
        assert_eq!(
            server.process(
                &partial_ack,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                5,
            ),
            Err(())
        );
        let retry_first = server.poll_block_upload(6).unwrap();
        let retry_second = server.poll_block_upload(7).unwrap();
        assert_eq!(retry_first[0], 1);
        assert_eq!(&retry_first[1..8], &od.long_data[7..14]);
        assert_eq!(retry_second[0], 0x82);
        assert_eq!(&retry_second[1..7], &od.long_data[14..20]);

        let final_ack = [0xA2, 2, 2, 0, 0, 0, 0, 0];
        server
            .process(
                &final_ack,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                8,
            )
            .unwrap();
        assert_eq!(resp[0], 0xC5, "end response with one unused byte");
        assert_eq!(
            u16::from_le_bytes([resp[1], resp[2]]),
            crc16_ccitt_update(0, &od.long_data)
        );
    }

    #[test]
    fn block_upload_rejects_invalid_block_sizes() {
        let mut server = SdoServer::new();
        let mut od = TestOd::new();
        let mut resp = [0u8; 8];
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let invalid_init = [0xA4, 0x01, 0x20, 0x00, 0, 0, 0, 0];
        server
            .process(
                &invalid_init,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .unwrap();
        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        assert_eq!(
            u32::from_le_bytes(resp[4..8].try_into().unwrap()),
            AbortCode::InvalidBlockSize as u32
        );

        let init = [0xA4, 0x01, 0x20, 0x00, 1, 0, 0, 0];
        server
            .process(
                &init,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                1,
            )
            .unwrap();
        let start = [0xA3, 0, 0, 0, 0, 0, 0, 0];
        assert!(server
            .process(
                &start,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                2,
            )
            .is_err());
        let _ = server.poll_block_upload(3).unwrap();

        let invalid_ack = [0xA2, 1, 0, 0, 0, 0, 0, 0];
        server
            .process(
                &invalid_ack,
                &mut od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                4,
            )
            .unwrap();
        assert_eq!(command_specifier(resp[0]), Scs::AbortTransfer as u8);
        assert_eq!(
            u32::from_le_bytes(resp[4..8].try_into().unwrap()),
            AbortCode::InvalidBlockSize as u32
        );
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

    #[test]
    fn pdo_cob_id_cannot_be_moved_onto_sync() {
        let raw = 0x123u32.to_le_bytes();
        assert_eq!(
            validate_pdo_cob_id_write(SdoCobIds::default(), 0x123, 0x1400, 1, &raw),
            Err(AbortCode::ParameterIncompatibility)
        );
        assert_eq!(
            validate_pdo_cob_id_write(SdoCobIds::default(), 0x080, 0x1400, 1, &raw),
            Ok(())
        );
    }

    #[test]
    fn monitored_heartbeat_cannot_be_moved_onto_sync() {
        let od = TestOd::new();
        let entry = ((5u32 << 16) | 100).to_le_bytes();
        assert_eq!(
            validate_heartbeat_consumer_write(
                &od,
                SdoCobIds::default(),
                0x705,
                1,
                0x1016,
                1,
                &entry,
            ),
            Err(AbortCode::ParameterIncompatibility)
        );
        assert_eq!(
            validate_heartbeat_consumer_write(
                &od,
                SdoCobIds::default(),
                0x706,
                1,
                0x1016,
                1,
                &entry,
            ),
            Ok(())
        );
    }
}
