//! Shared CAN bus — std implementation of the master application model.
//!
//! One task owns the physical transport and publishes every received frame
//! to the bus; any number of independent consumers subscribe with their own
//! bounded queue, optional publish-side filter, and overflow policy. See
//! `_Tasks/master-application-model.md` for the design discussion.
//!
//! Design decisions (2026-07-06):
//! - Frames are `Copy`; each accepting subscriber gets its own copy. Filters
//!   run against a reference *before* the copy, so rejected frames cost one
//!   closure call.
//! - TX is not part of the bus. [`TxQueue`] is a separate bounded queue with
//!   non-lossy (blocking) `send` — silently dropping a TX frame turns into
//!   an inscrutable SDO timeout.
//! - No transport lifecycle handling: when the adapter dies, the pump
//!   returns an error and the application is expected to tear down and be
//!   restarted by its supervisor.
//! - Per-subscriber FIFO in publish order is an API guarantee; SDO
//!   correctness depends on it.
//!
//! This module is `std`-only. A no-alloc sibling with the same API shape
//! (intrusive `'static` subscriptions, plain-`fn` filters) is planned for
//! embedded masters.

use std::boxed::Box;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, Weak};
use std::task::{Poll, Waker};
use std::time::{Duration, Instant};
use std::vec::Vec;

use crate::sdo::driver::AsyncCan;
use crate::transport::CanFrame;

/// Wakers of async tasks waiting on a state change. Multiple waiters are
/// supported (a `TxQueue` is `Clone`, a `Subscription` can be shared by
/// reference): every state change wakes all of them, winners take the
/// resource, losers re-register on their next poll. No fairness guarantee.
#[derive(Default)]
struct WakerSet {
    wakers: Mutex<Vec<Waker>>,
}

impl WakerSet {
    fn register(&self, waker: &Waker) {
        let mut wakers = self.wakers.lock().expect("waker set poisoned");
        if !wakers.iter().any(|w| w.will_wake(waker)) {
            wakers.push(waker.clone());
        }
    }

    fn wake_all(&self) {
        // Take the wakers out first — waking while holding the lock could
        // re-enter register() from an executor that polls inline.
        let wakers: Vec<Waker> =
            std::mem::take(&mut *self.wakers.lock().expect("waker set poisoned"));
        for w in wakers {
            w.wake();
        }
    }
}

/// Publish-side frame filter. Runs under the publish path against a
/// reference; the frame is copied into the subscription queue only when the
/// filter returns `true`.
pub type FrameFilter = Box<dyn Fn(&CanFrame) -> bool + Send + Sync>;

/// What to do when a subscription queue is full.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum OverflowPolicy {
    /// Drop the oldest queued frame to make room (latest state wins —
    /// right for monitoring TPDOs/heartbeats).
    #[default]
    DropOldest,
    /// Drop the incoming frame.
    DropNewest,
}

struct SubscriptionShared {
    queue: Mutex<VecDeque<CanFrame>>,
    ready: Condvar,
    rx_wakers: WakerSet,
    capacity: usize,
    policy: OverflowPolicy,
    overflow: AtomicU64,
    filter: Option<FrameFilter>,
}

impl SubscriptionShared {
    fn push(&self, frame: CanFrame) {
        let mut queue = self.queue.lock().expect("subscription queue poisoned");
        if queue.len() >= self.capacity {
            self.overflow.fetch_add(1, Ordering::Relaxed);
            match self.policy {
                OverflowPolicy::DropOldest => {
                    queue.pop_front();
                }
                OverflowPolicy::DropNewest => return,
            }
        }
        queue.push_back(frame);
        self.ready.notify_one();
        drop(queue);
        self.rx_wakers.wake_all();
    }
}

/// Handle to the shared bus. Cheap to clone; all clones publish to and
/// subscribe on the same bus.
#[derive(Clone, Default)]
pub struct SharedCanBus {
    subs: Arc<Mutex<Vec<Weak<SubscriptionShared>>>>,
}

impl SharedCanBus {
    pub fn new() -> Self {
        Self::default()
    }

    /// Subscribe with the default overflow policy and no filter.
    pub fn subscribe(&self, capacity: usize) -> Subscription {
        self.subscribe_with(capacity, OverflowPolicy::default(), None)
    }

    /// Subscribe with a publish-side filter (default overflow policy).
    pub fn subscribe_filtered(
        &self,
        capacity: usize,
        filter: impl Fn(&CanFrame) -> bool + Send + Sync + 'static,
    ) -> Subscription {
        self.subscribe_with(capacity, OverflowPolicy::default(), Some(Box::new(filter)))
    }

    pub fn subscribe_with(
        &self,
        capacity: usize,
        policy: OverflowPolicy,
        filter: Option<FrameFilter>,
    ) -> Subscription {
        assert!(capacity > 0, "subscription capacity must be non-zero");
        let shared = Arc::new(SubscriptionShared {
            queue: Mutex::new(VecDeque::with_capacity(capacity)),
            ready: Condvar::new(),
            rx_wakers: WakerSet::default(),
            capacity,
            policy,
            overflow: AtomicU64::new(0),
            filter,
        });
        self.subs
            .lock()
            .expect("bus subscriber list poisoned")
            .push(Arc::downgrade(&shared));
        Subscription { shared }
    }

    /// Publish a frame to every live subscriber whose filter accepts it.
    /// Dropped subscriptions are pruned as a side effect.
    pub fn publish(&self, frame: &CanFrame) {
        let mut subs = self.subs.lock().expect("bus subscriber list poisoned");
        subs.retain(|weak| {
            let Some(sub) = weak.upgrade() else {
                return false;
            };
            if sub.filter.as_ref().is_none_or(|f| f(frame)) {
                sub.push(*frame);
            }
            true
        });
    }

    /// Number of live subscriptions (prunes dead ones).
    pub fn subscriber_count(&self) -> usize {
        let mut subs = self.subs.lock().expect("bus subscriber list poisoned");
        subs.retain(|weak| weak.strong_count() > 0);
        subs.len()
    }
}

/// A subscription's receiving end. Dropping it unsubscribes.
pub struct Subscription {
    shared: Arc<SubscriptionShared>,
}

impl Subscription {
    /// Take the next frame without blocking.
    pub fn try_recv(&self) -> Option<CanFrame> {
        self.shared
            .queue
            .lock()
            .expect("subscription queue poisoned")
            .pop_front()
    }

    /// Block until a frame arrives.
    ///
    /// Note: there is no disconnect signal — if the pump is gone this blocks
    /// forever. Prefer [`recv_timeout`](Self::recv_timeout) in loops that
    /// must notice a dead bus.
    pub fn recv(&self) -> CanFrame {
        let mut queue = self
            .shared
            .queue
            .lock()
            .expect("subscription queue poisoned");
        loop {
            if let Some(frame) = queue.pop_front() {
                return frame;
            }
            queue = self
                .shared
                .ready
                .wait(queue)
                .expect("subscription queue poisoned");
        }
    }

    /// Block until a frame arrives or the timeout elapses.
    pub fn recv_timeout(&self, timeout: Duration) -> Option<CanFrame> {
        let deadline = Instant::now() + timeout;
        let mut queue = self
            .shared
            .queue
            .lock()
            .expect("subscription queue poisoned");
        loop {
            if let Some(frame) = queue.pop_front() {
                return Some(frame);
            }
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            let (guard, _) = self
                .shared
                .ready
                .wait_timeout(queue, deadline - now)
                .expect("subscription queue poisoned");
            queue = guard;
        }
    }

    /// Receive a frame, suspending the task while the queue is empty.
    ///
    /// Waker-backed: the publish path wakes waiting tasks, so this is safe on
    /// multitasking runtimes (tokio, ...) — no polling, no spinning. Several
    /// tasks may wait on the same subscription; each frame goes to exactly
    /// one of them (no fairness guarantee).
    ///
    /// Like [`recv`](Self::recv), there is no disconnect signal — race this
    /// against a timeout if you must notice a dead bus.
    pub async fn recv_async(&self) -> CanFrame {
        core::future::poll_fn(|cx| {
            if let Some(frame) = self.try_recv() {
                return Poll::Ready(frame);
            }
            self.shared.rx_wakers.register(cx.waker());
            // Re-check: a frame published between try_recv and register would
            // otherwise be missed (its wake_all ran before we registered).
            match self.try_recv() {
                Some(frame) => Poll::Ready(frame),
                None => Poll::Pending,
            }
        })
        .await
    }

    /// Frames dropped due to a full queue since the last call (resets the
    /// counter).
    pub fn take_overflow_count(&self) -> u64 {
        self.shared.overflow.swap(0, Ordering::Relaxed)
    }

    pub fn len(&self) -> usize {
        self.shared
            .queue
            .lock()
            .expect("subscription queue poisoned")
            .len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

// ---------------------------------------------------------------------------
// TX queue
// ---------------------------------------------------------------------------

struct TxShared {
    queue: Mutex<VecDeque<CanFrame>>,
    capacity: usize,
    space: Condvar,
    ready: Condvar,
    space_wakers: WakerSet,
}

/// Bounded transmit queue between producers and the pump. `send` blocks
/// while the queue is full — TX frames are never silently dropped.
#[derive(Clone)]
pub struct TxQueue {
    shared: Arc<TxShared>,
}

impl TxQueue {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "tx queue capacity must be non-zero");
        Self {
            shared: Arc::new(TxShared {
                queue: Mutex::new(VecDeque::with_capacity(capacity)),
                capacity,
                space: Condvar::new(),
                ready: Condvar::new(),
                space_wakers: WakerSet::default(),
            }),
        }
    }

    /// Enqueue a frame, blocking while the queue is full.
    pub fn send(&self, frame: &CanFrame) {
        let mut queue = self.shared.queue.lock().expect("tx queue poisoned");
        while queue.len() >= self.shared.capacity {
            queue = self.shared.space.wait(queue).expect("tx queue poisoned");
        }
        queue.push_back(*frame);
        self.shared.ready.notify_one();
    }

    /// Enqueue without blocking. Returns `false` if the queue is full.
    pub fn try_send(&self, frame: &CanFrame) -> bool {
        let mut queue = self.shared.queue.lock().expect("tx queue poisoned");
        if queue.len() >= self.shared.capacity {
            return false;
        }
        queue.push_back(*frame);
        self.shared.ready.notify_one();
        true
    }

    /// Enqueue a frame, suspending the task while the queue is full.
    ///
    /// Waker-backed counterpart of [`send`](Self::send): the pump's
    /// [`try_pop`](Self::try_pop) wakes waiting tasks when space frees up, so
    /// this is safe on multitasking runtimes — no polling, no spinning.
    /// Multiple tasks may wait for space (the queue is `Clone`); each freed
    /// slot goes to exactly one of them (no fairness guarantee).
    pub async fn send_async(&self, frame: &CanFrame) {
        core::future::poll_fn(|cx| {
            if self.try_send(frame) {
                return Poll::Ready(());
            }
            self.shared.space_wakers.register(cx.waker());
            // Re-check: a slot freed between try_send and register would
            // otherwise be missed (its wake_all ran before we registered).
            if self.try_send(frame) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Pump side: take the next frame to transmit, if any.
    pub fn try_pop(&self) -> Option<CanFrame> {
        let mut queue = self.shared.queue.lock().expect("tx queue poisoned");
        let frame = queue.pop_front();
        if frame.is_some() {
            self.shared.space.notify_one();
            drop(queue);
            self.shared.space_wakers.wake_all();
        }
        frame
    }

    pub fn len(&self) -> usize {
        self.shared.queue.lock().expect("tx queue poisoned").len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

// ---------------------------------------------------------------------------
// AsyncCan port over (TxQueue, Subscription)
// ---------------------------------------------------------------------------

/// [`AsyncCan`] adapter over a TX queue and a subscription, so `SdoDriver`
/// and generated EDS clients run unchanged on the shared bus.
///
/// Subscribe with a filter for the target's SDO responses, e.g.:
///
/// ```ignore
/// let response_cob = CobId::sdo_tx(target).raw();
/// let rx = bus.subscribe_filtered(8, move |f| f.raw_id() == response_cob);
/// let mut port = SubscriptionPort::new(&tx, &rx);
/// let driver = SdoDriver::new(target);
/// let val = block_on_with_timeout(driver.read_u32(0x1000, 0, &mut port), timeout)??;
/// ```
///
/// `receive`/`transmit` are waker-backed ([`Subscription::recv_async`],
/// [`TxQueue::send_async`]): idle waits suspend the task until the publish
/// path / pump wakes it, so the port is efficient on multitasking runtimes
/// (tokio, ...) and still works with simple polling executors
/// (`block_on_with_timeout`).
pub struct SubscriptionPort<'a> {
    tx: &'a TxQueue,
    rx: &'a Subscription,
}

impl<'a> SubscriptionPort<'a> {
    pub fn new(tx: &'a TxQueue, rx: &'a Subscription) -> Self {
        Self { tx, rx }
    }
}

impl AsyncCan for SubscriptionPort<'_> {
    type Error = core::convert::Infallible;

    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        self.tx.send_async(frame).await;
        Ok(())
    }

    async fn receive(&mut self) -> Result<CanFrame, Self::Error> {
        Ok(self.rx.recv_async().await)
    }
}

// ---------------------------------------------------------------------------
// Pump
// ---------------------------------------------------------------------------

/// Pump failure. There is no recovery path by design — tear down and let a
/// supervisor restart the application.
#[derive(Debug)]
pub enum PumpError<E> {
    /// Transport I/O error (adapter unplugged, bus-off, ...).
    Transport(E),
    /// A frame could not be transmitted within [`TX_STUCK_TIMEOUT`].
    TxStuck,
}

/// How long the pump retries a `WouldBlock`ing transmit before giving up.
pub const TX_STUCK_TIMEOUT: Duration = Duration::from_millis(500);

/// Drive the physical transport: publish received frames to the bus and
/// transmit frames from the TX queue, until `stop` is set or the transport
/// fails. This function owns the calling thread.
pub fn run_pump<T>(
    transport: &mut T,
    bus: &SharedCanBus,
    tx: &TxQueue,
    stop: &AtomicBool,
    idle_sleep: Duration,
) -> Result<(), PumpError<T::Error>>
where
    T: embedded_can::nb::Can<Frame = CanFrame>,
{
    while !stop.load(Ordering::Relaxed) {
        let mut busy = false;

        // RX: drain everything pending.
        loop {
            match transport.receive() {
                Ok(frame) => {
                    bus.publish(&frame);
                    busy = true;
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(e)) => return Err(PumpError::Transport(e)),
            }
        }

        // TX: drain the queue.
        while let Some(frame) = tx.try_pop() {
            let deadline = Instant::now() + TX_STUCK_TIMEOUT;
            loop {
                match transport.transmit(&frame) {
                    Ok(_) => break,
                    Err(nb::Error::WouldBlock) if Instant::now() < deadline => {
                        std::thread::sleep(Duration::from_micros(50));
                    }
                    Err(nb::Error::WouldBlock) => return Err(PumpError::TxStuck),
                    Err(nb::Error::Other(e)) => return Err(PumpError::Transport(e)),
                }
            }
            busy = true;
        }

        if !busy {
            std::thread::sleep(idle_sleep);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cobid::{CobId, NodeId};

    fn frame(id: u16) -> CanFrame {
        CanFrame::new(id, &[0xAA]).unwrap()
    }

    #[test]
    fn publish_fans_out_to_all_subscribers() {
        let bus = SharedCanBus::new();
        let a = bus.subscribe(4);
        let b = bus.subscribe(4);

        bus.publish(&frame(0x181));

        assert_eq!(a.try_recv().unwrap().raw_id(), 0x181);
        assert_eq!(b.try_recv().unwrap().raw_id(), 0x181);
        assert!(a.try_recv().is_none());
        assert!(b.try_recv().is_none());
    }

    #[test]
    fn filter_runs_before_copy() {
        let bus = SharedCanBus::new();
        let sdo_only = bus.subscribe_filtered(4, |f| f.raw_id() == 0x5A1);
        let all = bus.subscribe(4);

        bus.publish(&frame(0x181));
        bus.publish(&frame(0x5A1));

        assert_eq!(sdo_only.try_recv().unwrap().raw_id(), 0x5A1);
        assert!(sdo_only.try_recv().is_none());
        assert_eq!(all.len(), 2);
        // Filtered-out frames never count as overflow
        assert_eq!(sdo_only.take_overflow_count(), 0);
    }

    #[test]
    fn overflow_drop_oldest_keeps_latest() {
        let bus = SharedCanBus::new();
        let sub = bus.subscribe_with(2, OverflowPolicy::DropOldest, None);

        bus.publish(&frame(0x101));
        bus.publish(&frame(0x102));
        bus.publish(&frame(0x103));

        assert_eq!(sub.take_overflow_count(), 1);
        assert_eq!(sub.try_recv().unwrap().raw_id(), 0x102);
        assert_eq!(sub.try_recv().unwrap().raw_id(), 0x103);
    }

    #[test]
    fn overflow_drop_newest_keeps_earliest() {
        let bus = SharedCanBus::new();
        let sub = bus.subscribe_with(2, OverflowPolicy::DropNewest, None);

        bus.publish(&frame(0x101));
        bus.publish(&frame(0x102));
        bus.publish(&frame(0x103));

        assert_eq!(sub.take_overflow_count(), 1);
        assert_eq!(sub.try_recv().unwrap().raw_id(), 0x101);
        assert_eq!(sub.try_recv().unwrap().raw_id(), 0x102);
        assert!(sub.try_recv().is_none());
    }

    #[test]
    fn dropping_subscription_unsubscribes() {
        let bus = SharedCanBus::new();
        let a = bus.subscribe(4);
        let b = bus.subscribe(4);
        assert_eq!(bus.subscriber_count(), 2);

        drop(b);
        bus.publish(&frame(0x181));
        assert_eq!(bus.subscriber_count(), 1);
        assert_eq!(a.len(), 1);
    }

    #[test]
    fn recv_timeout_times_out_and_delivers() {
        let bus = SharedCanBus::new();
        let sub = bus.subscribe(4);

        assert!(sub.recv_timeout(Duration::from_millis(10)).is_none());

        let bus2 = bus.clone();
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(20));
            bus2.publish(&frame(0x181));
        });
        let got = sub.recv_timeout(Duration::from_secs(2));
        t.join().unwrap();
        assert_eq!(got.unwrap().raw_id(), 0x181);
    }

    /// Counts wakes; lets tests assert both "woken on progress" and, just as
    /// important, "NOT woken while idle" (no self-wake busy-spin).
    struct CountingWaker(std::sync::atomic::AtomicUsize);

    impl std::task::Wake for CountingWaker {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
        fn wake_by_ref(self: &Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn counting_waker() -> (Arc<CountingWaker>, Waker) {
        let counter = Arc::new(CountingWaker(std::sync::atomic::AtomicUsize::new(0)));
        let waker = Waker::from(counter.clone());
        (counter, waker)
    }

    #[test]
    fn recv_async_suspends_idle_and_wakes_on_publish() {
        use core::future::Future;
        use core::pin::pin;
        use core::task::Context;

        let bus = SharedCanBus::new();
        let sub = bus.subscribe(4);
        let (counter, waker) = counting_waker();
        let mut cx = Context::from_waker(&waker);

        let fut = sub.recv_async();
        let mut fut = pin!(fut);

        // Idle: Pending without waking itself — this is the no-busy-spin
        // guarantee. A self-waking future would have counter > 0 here.
        assert!(fut.as_mut().poll(&mut cx).is_pending());
        assert!(fut.as_mut().poll(&mut cx).is_pending());
        assert_eq!(counter.0.load(Ordering::SeqCst), 0);

        // Publish wakes the task, and the next poll completes.
        bus.publish(&frame(0x181));
        assert_eq!(counter.0.load(Ordering::SeqCst), 1);
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(f) => assert_eq!(f.raw_id(), 0x181),
            Poll::Pending => panic!("expected Ready after publish"),
        }
    }

    #[test]
    fn recv_async_multiple_waiters_each_get_a_frame() {
        use core::future::Future;
        use core::pin::pin;
        use core::task::Context;

        let bus = SharedCanBus::new();
        let sub = bus.subscribe(4);
        let (c1, w1) = counting_waker();
        let (c2, w2) = counting_waker();
        let mut cx1 = Context::from_waker(&w1);
        let mut cx2 = Context::from_waker(&w2);

        let fut1 = sub.recv_async();
        let fut2 = sub.recv_async();
        let mut fut1 = pin!(fut1);
        let mut fut2 = pin!(fut2);

        assert!(fut1.as_mut().poll(&mut cx1).is_pending());
        assert!(fut2.as_mut().poll(&mut cx2).is_pending());

        // One frame wakes all waiters; exactly one wins it, the other
        // re-registers and gets the second frame.
        bus.publish(&frame(0x201));
        assert!(c1.0.load(Ordering::SeqCst) > 0);
        assert!(c2.0.load(Ordering::SeqCst) > 0);

        let r1 = fut1.as_mut().poll(&mut cx1);
        let r2 = fut2.as_mut().poll(&mut cx2);
        assert!(
            r1.is_ready() ^ r2.is_ready(),
            "exactly one waiter should win the single frame"
        );

        // The losing waiter re-registered during its poll and gets the next frame.
        bus.publish(&frame(0x202));
        if r1.is_ready() {
            assert!(fut2.as_mut().poll(&mut cx2).is_ready());
        } else {
            assert!(fut1.as_mut().poll(&mut cx1).is_ready());
        }
    }

    #[test]
    fn send_async_suspends_when_full_and_wakes_on_pop() {
        use core::future::Future;
        use core::pin::pin;
        use core::task::Context;

        let tx = TxQueue::new(1);
        assert!(tx.try_send(&frame(1)));

        let (counter, waker) = counting_waker();
        let mut cx = Context::from_waker(&waker);
        let f2 = frame(2);
        let fut = tx.send_async(&f2);
        let mut fut = pin!(fut);

        // Queue full: Pending, and no self-wake while idle.
        assert!(fut.as_mut().poll(&mut cx).is_pending());
        assert_eq!(counter.0.load(Ordering::SeqCst), 0);

        // Pump frees a slot → waiter is woken and completes.
        assert_eq!(tx.try_pop().unwrap().raw_id(), 1);
        assert_eq!(counter.0.load(Ordering::SeqCst), 1);
        assert!(fut.as_mut().poll(&mut cx).is_ready());
        assert_eq!(tx.try_pop().unwrap().raw_id(), 2);
    }

    #[test]
    fn recv_async_works_on_tokio_style_runtime() {
        // Cross-thread smoke test with real thread wakeups: a std::thread
        // "runtime" that only re-polls when the waker fires.
        use std::sync::mpsc;

        struct ThreadWaker(mpsc::Sender<()>);
        impl std::task::Wake for ThreadWaker {
            fn wake(self: Arc<Self>) {
                let _ = self.0.send(());
            }
        }

        let bus = SharedCanBus::new();
        let sub = bus.subscribe(4);

        let handle = std::thread::spawn(move || {
            use core::future::Future;
            use core::pin::pin;
            use core::task::{Context, Poll};

            let (wake_tx, wake_rx) = mpsc::channel();
            let waker = Waker::from(Arc::new(ThreadWaker(wake_tx)));
            let mut cx = Context::from_waker(&waker);
            let fut = sub.recv_async();
            let mut fut = pin!(fut);
            loop {
                match fut.as_mut().poll(&mut cx) {
                    Poll::Ready(f) => return f.raw_id(),
                    // Block until woken — no polling loop, no sleep.
                    Poll::Pending => wake_rx
                        .recv_timeout(Duration::from_secs(5))
                        .expect("waker never fired"),
                }
            }
        });

        std::thread::sleep(Duration::from_millis(50));
        bus.publish(&frame(0x1B1));
        assert_eq!(handle.join().unwrap(), 0x1B1);
    }

    #[test]
    fn tx_queue_send_try_send_pop() {
        let tx = TxQueue::new(2);
        assert!(tx.try_send(&frame(1)));
        assert!(tx.try_send(&frame(2)));
        assert!(!tx.try_send(&frame(3)));

        assert_eq!(tx.try_pop().unwrap().raw_id(), 1);
        assert!(tx.try_send(&frame(3)));
        assert_eq!(tx.try_pop().unwrap().raw_id(), 2);
        assert_eq!(tx.try_pop().unwrap().raw_id(), 3);
        assert!(tx.try_pop().is_none());
    }

    /// End-to-end: SdoDriver over a SubscriptionPort, with an SdoServer
    /// answering "on the wire" between polls, plus unrelated bus noise.
    #[test]
    fn sdo_driver_over_subscription_port() {
        use crate::nmt::NmtState;
        use crate::od::*;
        use crate::sdo::driver::SdoDriver;
        use crate::sdo::server::SdoServer;

        struct MiniOd;
        static META: &[OdEntryMeta] = &[OdEntryMeta {
            index: 0x1000,
            subindex: 0,
            data_type: crate::datatypes::DataType::U32,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "device_type",
            max_size: None,
        }];
        impl ObjectDictionary for MiniOd {
            fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
                META.iter()
                    .find(|e| e.index == index && e.subindex == subindex)
            }
            fn read(&self, index: u16, _sub: u8, buf: &mut [u8]) -> Result<usize, OdError> {
                if index == 0x1000 {
                    buf[..4].copy_from_slice(&0xCAFE_u32.to_le_bytes());
                    Ok(4)
                } else {
                    Err(OdError::NotFound)
                }
            }
            fn write(&mut self, _: u16, _: u8, _: &[u8]) -> Result<(), OdError> {
                Err(OdError::ReadOnly)
            }
            fn sub_count(&self, _: u16) -> Option<u8> {
                Some(0)
            }
        }

        let target = NodeId::new(0x21).unwrap();
        let bus = SharedCanBus::new();
        let tx = TxQueue::new(8);
        let response_cob = CobId::sdo_tx(target).raw();
        let rx = bus.subscribe_filtered(8, move |f| f.raw_id() == response_cob);

        let driver = SdoDriver::new(target);
        let mut port = SubscriptionPort::new(&tx, &rx);
        let fut = driver.read_u32(0x1000, 0, &mut port);

        // Poll the future manually; between polls, play the wire: requests
        // from the TX queue go to an SdoServer, responses (plus noise) are
        // published on the bus.
        use core::future::Future;
        use core::pin::pin;
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        fn raw_waker() -> RawWaker {
            fn no_op(_: *const ()) {}
            fn clone(p: *const ()) -> RawWaker {
                RawWaker::new(p, &VTABLE)
            }
            const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        let waker = unsafe { Waker::from_raw(raw_waker()) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(fut);

        let mut server = SdoServer::new();
        let mut od = MiniOd;

        let val = loop {
            match fut.as_mut().poll(&mut cx) {
                Poll::Ready(v) => break v.unwrap(),
                Poll::Pending => {
                    while let Some(req) = tx.try_pop() {
                        // Noise the filter must reject
                        bus.publish(&CanFrame::new(0x1A1, &[0x05]).unwrap());

                        let mut req_data = [0u8; 8];
                        req_data.copy_from_slice(req.data());
                        let mut resp = [0u8; 8];
                        let mut events: crate::heapless::Deque<OdEvent, 16> =
                            crate::heapless::Deque::new();
                        if server
                            .process(
                                &req_data,
                                &mut od,
                                &mut resp,
                                &mut events,
                                NmtState::Operational,
                                0,
                            )
                            .is_ok()
                        {
                            bus.publish(&CanFrame::new(response_cob, &resp).unwrap());
                        }
                    }
                }
            }
        };

        assert_eq!(val, 0xCAFE);
        // The noise frames never reached the SDO subscription
        assert!(rx.is_empty());
    }
}
