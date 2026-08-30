use std::collections::VecDeque;
use std::time::{Duration, Instant};
use std::vec::Vec;

use canopen_core::cobid::NodeId;
use canopen_core::sdo::{AsyncCan, SdoAbort, SdoDriver, SdoError};
use canopen_core::time::Clock;

/// Maximum transfer size of the currently used [`SdoDriver`].
pub const SDO_MAX_DATA: usize = 256;

/// One complete SDO operation requested by a middleware service.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SdoCall {
    pub target: NodeId,
    pub index: u16,
    pub subindex: u8,
    pub transfer: SdoTransfer,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SdoTransfer {
    Upload { max_len: usize },
    Download(Vec<u8>),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SdoReply {
    Upload(Vec<u8>),
    Downloaded,
}

/// Stable failure vocabulary for service response mapping.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SdoFailure {
    Aborted(SdoAbort),
    Timeout,
    Protocol,
    Transport,
    TooLarge,
    QueueExpired,
    ShuttingDown,
}

impl SdoCall {
    /// Execute this call with the stack's high-level SDO driver.
    ///
    /// The timeout is a protocol-response deadline, distinct from a
    /// [`ServiceQueue`]'s pre-start queue age. Transport details are reduced to
    /// [`SdoFailure::Transport`] so middleware response schemas do not depend
    /// on a particular CAN adapter error type.
    pub async fn execute<E: core::fmt::Debug>(
        &self,
        can: &mut impl AsyncCan<Error = E>,
        timeout_us: u64,
        clock: &impl Clock,
    ) -> Result<SdoReply, SdoFailure> {
        let driver = SdoDriver::new(self.target);
        match &self.transfer {
            SdoTransfer::Upload { max_len } => {
                if *max_len > SDO_MAX_DATA {
                    return Err(SdoFailure::TooLarge);
                }
                let mut data = vec![0; *max_len];
                let actual = driver
                    .upload_timed(self.index, self.subindex, &mut data, can, timeout_us, clock)
                    .await
                    .map_err(map_sdo_error)?;
                if actual > *max_len {
                    return Err(SdoFailure::TooLarge);
                }
                data.truncate(actual);
                Ok(SdoReply::Upload(data))
            }
            SdoTransfer::Download(data) => {
                if data.len() > SDO_MAX_DATA {
                    return Err(SdoFailure::TooLarge);
                }
                driver
                    .download_timed(self.index, self.subindex, data, can, timeout_us, clock)
                    .await
                    .map_err(map_sdo_error)?;
                Ok(SdoReply::Downloaded)
            }
        }
    }
}

fn map_sdo_error<E: core::fmt::Debug>(error: SdoError<E>) -> SdoFailure {
    match error {
        SdoError::Aborted(code) => SdoFailure::Aborted(code),
        SdoError::ProtocolError => SdoFailure::Protocol,
        SdoError::Transport(_) => SdoFailure::Transport,
        SdoError::Timeout => SdoFailure::Timeout,
        SdoError::TooLarge => SdoFailure::TooLarge,
    }
}

/// Request/response conversion around an SDO operation.
///
/// The context returned by `start` is retained by the caller until completion,
/// allowing a response mapper to preserve request-specific information without
/// global state.
pub struct ServiceToSdo<Start, Finish> {
    start: Start,
    finish: Finish,
}

impl<Start, Finish> ServiceToSdo<Start, Finish> {
    pub const fn new(start: Start, finish: Finish) -> Self {
        Self { start, finish }
    }

    pub fn start<Request, Context, Response>(
        &self,
        request: Request,
    ) -> Result<(SdoCall, Context), Response>
    where
        Start: Fn(Request) -> Result<(SdoCall, Context), Response>,
    {
        (self.start)(request)
    }

    pub fn finish<Context, Response>(
        &self,
        context: Context,
        outcome: Result<SdoReply, SdoFailure>,
    ) -> Response
    where
        Finish: Fn(Context, Result<SdoReply, SdoFailure>) -> Response,
    {
        (self.finish)(context, outcome)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueueDiscipline {
    /// If another call is queued or active, reject the new request immediately.
    RejectConcurrent,
    /// Queue up to `capacity` waiting requests in FIFO order.
    Fifo { capacity: usize },
    /// Queue up to `capacity` requests; start the highest numeric priority,
    /// preserving FIFO order among requests with equal priority.
    Priority { capacity: usize },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ServiceQueueConfig {
    pub discipline: QueueDiscipline,
    /// Maximum time before an unstarted request is returned as expired.
    pub max_queue_age: Duration,
}

struct Queued<T> {
    value: T,
    enqueued_at: Instant,
    priority: u8,
    generation: u64,
}

/// An SDO service request that owns the channel until completed.
#[derive(Debug)]
pub struct ActiveService<T> {
    pub value: T,
    pub enqueued_at: Instant,
    pub started_at: Instant,
    generation: u64,
}

impl<T> ActiveService<T> {
    pub const fn generation(&self) -> u64 {
        self.generation
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EnqueueError<T> {
    Busy(T),
    Full(T),
}

#[derive(Debug)]
pub enum ServicePoll<T> {
    Ready(ActiveService<T>),
    /// Returned to the adapter so it can complete the middleware service with
    /// `SdoFailure::QueueExpired`; expiration never silently loses a caller.
    Expired(T),
    Busy,
    Empty,
}

/// Per-SDO-channel serialization and bounded waiting.
///
/// This queue intentionally does not impose an active-transfer timeout. That
/// timeout belongs to `SdoDriver`/the transport runner, which is the only layer
/// able to cancel the protocol transfer safely. A queue timeout only applies
/// before a request starts.
pub struct ServiceQueue<T> {
    config: ServiceQueueConfig,
    queued: VecDeque<Queued<T>>,
    active_generation: Option<u64>,
    next_generation: u64,
}

impl<T> ServiceQueue<T> {
    pub fn new(config: ServiceQueueConfig) -> Self {
        assert!(
            !matches!(
                config.discipline,
                QueueDiscipline::Fifo { capacity: 0 } | QueueDiscipline::Priority { capacity: 0 }
            ),
            "queued service capacity must be non-zero"
        );
        Self {
            config,
            queued: VecDeque::new(),
            active_generation: None,
            next_generation: 1,
        }
    }

    pub fn enqueue(&mut self, value: T, now: Instant) -> Result<u64, EnqueueError<T>> {
        self.enqueue_with_priority(value, now, 0)
    }

    /// Enqueue with an application-defined priority. `priority` is considered
    /// only by [`QueueDiscipline::Priority`].
    pub fn enqueue_with_priority(
        &mut self,
        value: T,
        now: Instant,
        priority: u8,
    ) -> Result<u64, EnqueueError<T>> {
        match self.config.discipline {
            QueueDiscipline::RejectConcurrent
                if self.active_generation.is_some() || !self.queued.is_empty() =>
            {
                return Err(EnqueueError::Busy(value));
            }
            QueueDiscipline::Fifo { capacity } | QueueDiscipline::Priority { capacity }
                if self.queued.len() >= capacity =>
            {
                return Err(EnqueueError::Full(value));
            }
            QueueDiscipline::RejectConcurrent
            | QueueDiscipline::Fifo { .. }
            | QueueDiscipline::Priority { .. } => {}
        }
        let generation = self.next_generation;
        self.next_generation = self
            .next_generation
            .checked_add(1)
            .expect("service generation exhausted");
        self.queued.push_back(Queued {
            value,
            enqueued_at: now,
            priority,
            generation,
        });
        Ok(generation)
    }

    /// Poll once. Expired requests are returned one at a time before later
    /// requests may start, preserving FIFO response ordering.
    pub fn poll(&mut self, now: Instant) -> ServicePoll<T> {
        if self.active_generation.is_some() {
            return ServicePoll::Busy;
        }
        let Some(front) = self.queued.front() else {
            return ServicePoll::Empty;
        };
        if now.saturating_duration_since(front.enqueued_at) >= self.config.max_queue_age {
            return ServicePoll::Expired(self.queued.pop_front().unwrap().value);
        }
        let index = match self.config.discipline {
            QueueDiscipline::RejectConcurrent | QueueDiscipline::Fifo { .. } => 0,
            QueueDiscipline::Priority { .. } => {
                let mut selected = 0;
                let mut selected_priority = self.queued[0].priority;
                for (index, queued) in self.queued.iter().enumerate().skip(1) {
                    if queued.priority > selected_priority {
                        selected = index;
                        selected_priority = queued.priority;
                    }
                }
                selected
            }
        };
        let queued = self.queued.remove(index).unwrap();
        self.active_generation = Some(queued.generation);
        ServicePoll::Ready(ActiveService {
            value: queued.value,
            enqueued_at: queued.enqueued_at,
            started_at: now,
            generation: queued.generation,
        })
    }

    /// Release the channel after the SDO runner completed or safely cancelled
    /// the exact active operation. Stale tokens cannot release a newer call.
    pub fn complete(&mut self, generation: u64) -> bool {
        if self.active_generation == Some(generation) {
            self.active_generation = None;
            true
        } else {
            false
        }
    }

    pub fn is_busy(&self) -> bool {
        self.active_generation.is_some()
    }

    pub fn queued_len(&self) -> usize {
        self.queued.len()
    }

    /// Drain requests that never started, e.g. on adapter shutdown. The active
    /// request remains owned by its runner and must be completed separately.
    pub fn drain_queued(&mut self) -> impl Iterator<Item = T> + '_ {
        self.queued.drain(..).map(|queued| queued.value)
    }

    /// Cancel selected requests that have not started, returning them in
    /// original enqueue order so every middleware caller can be completed.
    /// The active protocol transfer is never touched.
    pub fn cancel_queued(&mut self, mut predicate: impl FnMut(&T) -> bool) -> Vec<T> {
        let mut kept = VecDeque::with_capacity(self.queued.len());
        let mut cancelled = Vec::new();
        while let Some(queued) = self.queued.pop_front() {
            if predicate(&queued.value) {
                cancelled.push(queued.value);
            } else {
                kept.push_back(queued);
            }
        }
        self.queued = kept;
        cancelled
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::future::{ready, Future};
    use std::task::{Context, Poll, Waker};

    use canopen_core::transport::CanFrame;

    struct FixedClock;

    impl Clock for FixedClock {
        fn now_us(&self) -> u64 {
            0
        }
    }

    struct ScriptedCan {
        transmitted: Vec<CanFrame>,
        received: VecDeque<CanFrame>,
    }

    impl AsyncCan for ScriptedCan {
        type Error = core::convert::Infallible;

        fn transmit(&mut self, frame: &CanFrame) -> impl Future<Output = Result<(), Self::Error>> {
            self.transmitted.push(*frame);
            ready(Ok(()))
        }

        fn receive(&mut self) -> impl Future<Output = Result<CanFrame, Self::Error>> {
            ready(Ok(self.received.pop_front().expect("script exhausted")))
        }
    }

    fn complete_now<F: Future>(future: F) -> F::Output {
        let mut future = std::pin::pin!(future);
        let mut context = Context::from_waker(Waker::noop());
        match future.as_mut().poll(&mut context) {
            Poll::Ready(output) => output,
            Poll::Pending => panic!("scripted I/O should complete synchronously"),
        }
    }

    fn config(discipline: QueueDiscipline) -> ServiceQueueConfig {
        ServiceQueueConfig {
            discipline,
            max_queue_age: Duration::from_millis(100),
        }
    }

    #[test]
    fn reject_concurrent_is_immediate_and_returns_request() {
        let now = Instant::now();
        let mut queue = ServiceQueue::new(config(QueueDiscipline::RejectConcurrent));
        queue.enqueue("one", now).unwrap();
        assert_eq!(queue.enqueue("two", now), Err(EnqueueError::Busy("two")));
    }

    #[test]
    fn fifo_is_bounded_and_one_call_is_active() {
        let now = Instant::now();
        let mut queue = ServiceQueue::new(config(QueueDiscipline::Fifo { capacity: 2 }));
        queue.enqueue(1, now).unwrap();
        queue.enqueue(2, now).unwrap();
        assert_eq!(queue.enqueue(3, now), Err(EnqueueError::Full(3)));

        let first = match queue.poll(now) {
            ServicePoll::Ready(call) => call,
            _ => panic!("first request should start"),
        };
        assert!(matches!(queue.poll(now), ServicePoll::Busy));
        assert!(queue.complete(first.generation()));
        assert!(matches!(queue.poll(now), ServicePoll::Ready(call) if call.value == 2));
    }

    #[test]
    fn queued_request_expires_at_exact_boundary_and_is_returned() {
        let now = Instant::now();
        let mut queue = ServiceQueue::new(config(QueueDiscipline::Fifo { capacity: 2 }));
        queue.enqueue("request", now).unwrap();
        assert!(matches!(
            queue.poll(now + Duration::from_millis(100)),
            ServicePoll::Expired("request")
        ));
        assert!(matches!(queue.poll(now), ServicePoll::Empty));
    }

    #[test]
    fn stale_completion_cannot_release_a_new_transfer() {
        let now = Instant::now();
        let mut queue = ServiceQueue::new(config(QueueDiscipline::Fifo { capacity: 2 }));
        let old_generation = queue.enqueue(1, now).unwrap();
        let first = match queue.poll(now) {
            ServicePoll::Ready(call) => call,
            _ => unreachable!(),
        };
        assert_eq!(old_generation, first.generation());
        assert!(queue.complete(first.generation()));

        queue.enqueue(2, now).unwrap();
        let second = match queue.poll(now) {
            ServicePoll::Ready(call) => call,
            _ => unreachable!(),
        };
        assert!(!queue.complete(first.generation()));
        assert!(queue.is_busy());
        assert!(queue.complete(second.generation()));
    }

    #[test]
    fn priority_queue_prefers_control_but_is_fifo_within_a_priority() {
        let now = Instant::now();
        let mut queue = ServiceQueue::new(config(QueueDiscipline::Priority { capacity: 4 }));
        queue.enqueue_with_priority("inventory", now, 0).unwrap();
        queue.enqueue_with_priority("control one", now, 10).unwrap();
        queue.enqueue_with_priority("control two", now, 10).unwrap();

        let first = match queue.poll(now) {
            ServicePoll::Ready(call) => call,
            _ => unreachable!(),
        };
        assert_eq!(first.value, "control one");
        assert!(queue.complete(first.generation()));
        let second = match queue.poll(now) {
            ServicePoll::Ready(call) => call,
            _ => unreachable!(),
        };
        assert_eq!(second.value, "control two");
    }

    #[test]
    fn cancellation_returns_waiting_callers_without_touching_active() {
        let now = Instant::now();
        let mut queue = ServiceQueue::new(config(QueueDiscipline::Fifo { capacity: 4 }));
        queue.enqueue(1, now).unwrap();
        queue.enqueue(2, now).unwrap();
        queue.enqueue(3, now).unwrap();
        let active = match queue.poll(now) {
            ServicePoll::Ready(call) => call,
            _ => unreachable!(),
        };
        assert_eq!(queue.cancel_queued(|value| *value == 2), vec![2]);
        assert!(queue.is_busy());
        assert!(queue.complete(active.generation()));
        assert!(matches!(queue.poll(now), ServicePoll::Ready(call) if call.value == 3));
    }

    #[test]
    fn service_mapping_preserves_context_and_failures() {
        let node = NodeId::new(4).unwrap();
        let mapping = ServiceToSdo::new(
            move |value: u8| {
                Ok::<_, &'static str>((
                    SdoCall {
                        target: node,
                        index: 0x2000,
                        subindex: 3,
                        transfer: SdoTransfer::Download(vec![value]),
                    },
                    value,
                ))
            },
            |requested, result| (requested, result),
        );
        let (call, context) = mapping.start(7).unwrap();
        assert_eq!(call.transfer, SdoTransfer::Download(vec![7]));
        assert_eq!(
            mapping.finish(context, Err(SdoFailure::Timeout)),
            (7, Err(SdoFailure::Timeout))
        );
    }

    #[test]
    fn sdo_call_executes_upload_and_preserves_exact_width() {
        let node = NodeId::new(4).unwrap();
        let mut can = ScriptedCan {
            transmitted: Vec::new(),
            received: VecDeque::from([CanFrame::new(
                0x584,
                &[0x4b, 0x00, 0x20, 3, 0x34, 0x12, 0, 0],
            )
            .unwrap()]),
        };
        let call = SdoCall {
            target: node,
            index: 0x2000,
            subindex: 3,
            transfer: SdoTransfer::Upload { max_len: 2 },
        };

        assert_eq!(
            complete_now(call.execute(&mut can, 1_000_000, &FixedClock)),
            Ok(SdoReply::Upload(vec![0x34, 0x12]))
        );
        assert_eq!(can.transmitted.len(), 1);
        assert_eq!(can.transmitted[0].raw_id(), 0x604);
        assert_eq!(
            can.transmitted[0].data(),
            &[0x40, 0x00, 0x20, 3, 0, 0, 0, 0]
        );
    }

    #[test]
    fn oversized_sdo_is_rejected_before_can_transmit() {
        let mut can = ScriptedCan {
            transmitted: Vec::new(),
            received: VecDeque::new(),
        };
        let call = SdoCall {
            target: NodeId::new(4).unwrap(),
            index: 0x2000,
            subindex: 0,
            transfer: SdoTransfer::Download(vec![0; SDO_MAX_DATA + 1]),
        };
        assert_eq!(
            complete_now(call.execute(&mut can, 1_000_000, &FixedClock)),
            Err(SdoFailure::TooLarge)
        );
        assert!(can.transmitted.is_empty());
    }

    #[test]
    fn sdo_call_preserves_unknown_abort_code_for_service_response() {
        let mut can = ScriptedCan {
            transmitted: Vec::new(),
            received: VecDeque::from([CanFrame::new(
                0x584,
                &[0x80, 0x00, 0x20, 0, 0x42, 0x00, 0x00, 0x0f],
            )
            .unwrap()]),
        };
        let call = SdoCall {
            target: NodeId::new(4).unwrap(),
            index: 0x2000,
            subindex: 0,
            transfer: SdoTransfer::Upload { max_len: 1 },
        };
        let error = complete_now(call.execute(&mut can, 1_000_000, &FixedClock)).unwrap_err();
        assert!(matches!(error, SdoFailure::Aborted(code) if code.raw() == 0x0f00_0042));
    }
}
