//! Sans-IO SDO request engine.
//!
//! Wraps [`SdoClient`] with a request queue, a timeout, and in/out frame
//! buffers so the same code drives transfers on both targets: the native
//! backend owns a CAN thread, the wasm backend is single-threaded and
//! callback-driven, and neither can use `canopen_core::bus` (std-only,
//! thread/waker backed). The engine touches no transport at all — the caller
//! feeds it received frames and transmits whatever it hands back.

use canopen_core::cobid::{CobId, NodeId};
use canopen_core::sdo::client::{SdoClient, SdoClientResult};
use canopen_core::transport::CanFrame;
use std::collections::VecDeque;
use web_time::{Duration, Instant};

/// A silent server must not wedge the queue behind it forever.
const TRANSFER_TIMEOUT: Duration = Duration::from_millis(500);

/// Max segmented transfer size. 256 is the `SdoClient` default and covers
/// every object a device panel realistically reads (strings, small domains).
const SDO_BUF: usize = 256;

#[derive(Debug, Clone)]
pub struct SdoRequest {
    pub request_id: u64,
    pub node_id: u8,
    pub index: u16,
    pub subindex: u8,
    /// `None` = upload (read), `Some(bytes)` = download (write).
    pub write: Option<Vec<u8>>,
}

impl SdoRequest {
    pub fn read(request_id: u64, node_id: u8, index: u16, subindex: u8) -> Self {
        Self {
            request_id,
            node_id,
            index,
            subindex,
            write: None,
        }
    }

    pub fn write(request_id: u64, node_id: u8, index: u16, subindex: u8, data: Vec<u8>) -> Self {
        Self {
            request_id,
            node_id,
            index,
            subindex,
            write: Some(data),
        }
    }

    pub fn is_write(&self) -> bool {
        self.write.is_some()
    }
}

/// Result of one request. Reads carry the uploaded bytes; writes carry an
/// empty vec on success.
#[derive(Debug, Clone)]
pub struct SdoOutcome {
    pub request_id: u64,
    pub node_id: u8,
    pub index: u16,
    pub subindex: u8,
    pub was_write: bool,
    pub result: Result<Vec<u8>, String>,
}

struct Active {
    request: SdoRequest,
    client: SdoClient<SDO_BUF>,
    started: Instant,
}

#[derive(Default)]
pub struct SdoEngine {
    priority: VecDeque<SdoRequest>,
    queue: VecDeque<SdoRequest>,
    active: Option<Active>,
    tx: VecDeque<CanFrame>,
    done: VecDeque<SdoOutcome>,
}

impl SdoEngine {
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue a transfer. `SdoClient` tracks one at a time, so a panel polling
    /// several objects per tick relies on this queue rather than racing them.
    pub fn submit(&mut self, request: SdoRequest) {
        self.queue.push_back(request);
    }

    /// Queue a control-path transfer ahead of ordinary polling, while keeping
    /// priority requests FIFO relative to one another. The active transfer is
    /// allowed to finish so a late response cannot be mistaken for this one.
    pub fn submit_priority(&mut self, request: SdoRequest) {
        self.priority.push_back(request);
    }

    /// Cancel queued transfers for `node_id`. The active transfer is retained
    /// for protocol safety; it is the only request that can remain ahead of a
    /// subsequently submitted priority command.
    pub fn cancel_queued(&mut self, node_id: u8) {
        let mut cancelled = Vec::new();
        self.priority.retain(|request| {
            if request.node_id == node_id {
                cancelled.push(request.clone());
                false
            } else {
                true
            }
        });
        self.queue.retain(|request| {
            if request.node_id == node_id {
                cancelled.push(request.clone());
                false
            } else {
                true
            }
        });
        for request in cancelled {
            self.finish(request, Err("cancelled".to_string()));
        }
    }

    /// Frames to transmit, oldest first. Drain fully every loop iteration.
    pub fn next_tx(&mut self) -> Option<CanFrame> {
        self.tx.pop_front()
    }

    /// Completed transfers, oldest first.
    pub fn next_done(&mut self) -> Option<SdoOutcome> {
        self.done.pop_front()
    }

    /// Start the next queued transfer if idle, and fail the active one if it
    /// has gone quiet. Call once per loop iteration.
    pub fn tick(&mut self, now: Instant) {
        if let Some(active) = &self.active {
            if now.duration_since(active.started) > TRANSFER_TIMEOUT {
                let active = self.active.take().expect("checked above");
                self.finish(active.request, Err("timeout".to_string()));
            }
        }

        if self.active.is_some() {
            return;
        }

        while let Some(request) = self.priority.pop_front().or_else(|| self.queue.pop_front()) {
            let Some(node) = NodeId::new(request.node_id) else {
                self.finish(request, Err("invalid node id".to_string()));
                continue;
            };

            let mut client = SdoClient::<SDO_BUF>::new(node);
            let frame = match &request.write {
                None => client.start_upload(request.index, request.subindex),
                Some(data) => match client.start_download(request.index, request.subindex, data) {
                    Ok(frame) => frame,
                    Err(()) => {
                        self.finish(
                            request,
                            Err(format!("value larger than {SDO_BUF}-byte transfer buffer")),
                        );
                        continue;
                    }
                },
            };

            self.tx.push_back(frame);
            self.active = Some(Active {
                request,
                client,
                started: now,
            });
            return;
        }
    }

    /// Feed every received frame. Non-SDO frames and responses addressed to
    /// another node are ignored, so the caller can hand over the whole stream.
    pub fn on_frame(&mut self, frame: &CanFrame) {
        let Some(active) = &mut self.active else {
            return;
        };

        let Some(node) = NodeId::new(active.request.node_id) else {
            return;
        };
        if frame.raw_id() != CobId::sdo_tx(node).raw() {
            return;
        }

        // A short SDO response is malformed; the spec fixes the frame at 8
        // bytes. Pad rather than drop so a sloppy server still completes.
        let mut response = [0u8; 8];
        let data = frame.data();
        let len = data.len().min(8);
        response[..len].copy_from_slice(&data[..len]);

        match active.client.process_response(&response) {
            SdoClientResult::SendNext(next) => {
                self.tx.push_back(next);
                // Progress resets the clock: a long segmented transfer must
                // not trip the timeout that exists to catch a *silent* server.
                active.started = Instant::now();
            }
            SdoClientResult::UploadComplete { data_len } => {
                let active = self.active.take().expect("checked above");
                let bytes = active.client.data()[..data_len].to_vec();
                self.finish(active.request, Ok(bytes));
            }
            SdoClientResult::DownloadComplete => {
                let active = self.active.take().expect("checked above");
                self.finish(active.request, Ok(Vec::new()));
            }
            SdoClientResult::Aborted(code) => {
                let active = self.active.take().expect("checked above");
                self.finish(active.request, Err(format!("abort {code:?}")));
            }
            SdoClientResult::Error => {
                let active = self.active.take().expect("checked above");
                self.finish(active.request, Err("protocol error".to_string()));
            }
            // Another client's rejected request on the same channel.
            SdoClientResult::IgnoredAbort => {}
        }
    }

    /// Drop every queued and active transfer, e.g. on disconnect, so results
    /// from a previous session cannot surface against a new one.
    pub fn clear(&mut self) {
        self.priority.clear();
        self.queue.clear();
        self.active = None;
        self.tx.clear();
        self.done.clear();
    }

    fn finish(&mut self, request: SdoRequest, result: Result<Vec<u8>, String>) {
        self.done.push_back(SdoOutcome {
            request_id: request.request_id,
            node_id: request.node_id,
            index: request.index,
            subindex: request.subindex,
            was_write: request.is_write(),
            result,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(request_id: u64, node_id: u8) -> SdoRequest {
        SdoRequest::read(request_id, node_id, 0x6041, 0)
    }

    fn finish_active(engine: &mut SdoEngine) {
        let active = engine.active.take().expect("an active request");
        engine.finish(active.request, Ok(Vec::new()));
    }

    #[test]
    fn priority_is_fifo_ahead_of_polling() {
        let now = Instant::now();
        let mut engine = SdoEngine::new();
        engine.submit(read(1, 1));
        engine.submit(read(2, 1));
        engine.submit_priority(read(3, 1));
        engine.submit_priority(read(4, 1));

        for expected in [3, 4, 1, 2] {
            engine.tick(now);
            assert_eq!(
                engine
                    .active
                    .as_ref()
                    .map(|active| active.request.request_id),
                Some(expected)
            );
            finish_active(&mut engine);
        }
    }

    #[test]
    fn cancelling_a_node_keeps_other_nodes_and_reports_completion() {
        let mut engine = SdoEngine::new();
        engine.submit(read(1, 1));
        engine.submit(read(2, 2));
        engine.submit_priority(read(3, 1));

        engine.cancel_queued(1);

        let mut cancelled = Vec::new();
        while let Some(outcome) = engine.next_done() {
            assert_eq!(outcome.result, Err("cancelled".to_string()));
            cancelled.push(outcome.request_id);
        }
        cancelled.sort_unstable();
        assert_eq!(cancelled, vec![1, 3]);

        engine.tick(Instant::now());
        assert_eq!(
            engine
                .active
                .as_ref()
                .map(|active| active.request.request_id),
            Some(2)
        );
    }

    #[test]
    fn cancellation_does_not_preempt_the_active_protocol_transfer() {
        let mut engine = SdoEngine::new();
        engine.submit(read(1, 1));
        engine.tick(Instant::now());

        engine.cancel_queued(1);

        assert_eq!(
            engine
                .active
                .as_ref()
                .map(|active| active.request.request_id),
            Some(1)
        );
    }
}
