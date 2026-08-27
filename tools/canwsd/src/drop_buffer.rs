use std::collections::VecDeque;
use std::sync::Mutex;

use tokio::sync::Notify;

/// Result of draining a [`DropBuffer`]. Loss is surfaced before any item that
/// arrived after the buffer was cleared.
pub enum BufferEvent<T> {
    Item(T),
    Overflow(u64),
    Closed(String),
}

struct State<T> {
    items: VecDeque<T>,
    dropped: u64,
    closed: Option<String>,
}

/// A bounded queue for diagnostic streams. Once full, the entire stale backlog
/// is discarded and the next drain reports exactly how many items were lost.
pub struct DropBuffer<T> {
    capacity: usize,
    state: Mutex<State<T>>,
    ready: Notify,
}

impl<T> DropBuffer<T> {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0);
        Self {
            capacity,
            state: Mutex::new(State {
                items: VecDeque::with_capacity(capacity),
                dropped: 0,
                closed: None,
            }),
            ready: Notify::new(),
        }
    }

    pub fn push(&self, item: T) {
        {
            let mut state = self.state.lock().unwrap();
            if state.items.len() >= self.capacity {
                state.dropped = state.dropped.saturating_add(state.items.len() as u64);
                state.items.clear();
            }
            state.items.push_back(item);
        }
        self.ready.notify_one();
    }

    pub fn close(&self, error: impl Into<String>) {
        self.state.lock().unwrap().closed = Some(error.into());
        self.ready.notify_one();
    }

    pub async fn next(&self) -> BufferEvent<T> {
        loop {
            // A push racing the state check leaves a stored Notify permit; the
            // loop always re-checks the queue after consuming that permit.
            let notified = self.ready.notified();
            {
                let mut state = self.state.lock().unwrap();
                if state.dropped > 0 {
                    let dropped = state.dropped;
                    state.dropped = 0;
                    return BufferEvent::Overflow(dropped);
                }
                if let Some(item) = state.items.pop_front() {
                    return BufferEvent::Item(item);
                }
                if let Some(error) = state.closed.take() {
                    return BufferEvent::Closed(error);
                }
            }
            notified.await;
        }
    }
}

#[cfg(test)]
mod tests {
    use std::future::{Future, poll_fn};
    use std::task::Poll;

    use super::*;

    #[tokio::test]
    async fn overflow_is_reported_before_fresh_items() {
        let buffer = DropBuffer::new(2);
        buffer.push(1);
        buffer.push(2);
        buffer.push(3);

        assert!(matches!(buffer.next().await, BufferEvent::Overflow(2)));
        assert!(matches!(buffer.next().await, BufferEvent::Item(3)));
    }

    #[tokio::test]
    async fn repeated_clears_are_aggregated() {
        let buffer = DropBuffer::new(2);
        for item in 0..5 {
            buffer.push(item);
        }

        assert!(matches!(buffer.next().await, BufferEvent::Overflow(4)));
        assert!(matches!(buffer.next().await, BufferEvent::Item(4)));
    }

    #[tokio::test]
    async fn close_follows_buffered_items() {
        let buffer = DropBuffer::new(2);
        buffer.push(1);
        buffer.close("gone");

        assert!(matches!(buffer.next().await, BufferEvent::Item(1)));
        assert!(matches!(buffer.next().await, BufferEvent::Closed(error) if error == "gone"));
    }

    #[tokio::test]
    async fn close_follows_overflow_and_fresh_items() {
        let buffer = DropBuffer::new(2);
        buffer.push(1);
        buffer.push(2);
        buffer.push(3);
        buffer.close("gone");

        assert!(matches!(buffer.next().await, BufferEvent::Overflow(2)));
        assert!(matches!(buffer.next().await, BufferEvent::Item(3)));
        assert!(matches!(buffer.next().await, BufferEvent::Closed(error) if error == "gone"));
    }

    #[tokio::test]
    async fn waiting_consumer_is_woken_by_concurrent_push() {
        let buffer = std::sync::Arc::new(DropBuffer::new(2));
        let consumer_buffer = buffer.clone();
        let (waiting_tx, waiting_rx) = tokio::sync::oneshot::channel();
        let consumer = tokio::spawn(async move {
            let mut next = Box::pin(consumer_buffer.next());
            let mut waiting_tx = Some(waiting_tx);
            poll_fn(|cx| match next.as_mut().poll(cx) {
                Poll::Pending => {
                    waiting_tx.take().unwrap().send(()).unwrap();
                    Poll::Ready(())
                }
                Poll::Ready(_) => panic!("empty buffer unexpectedly produced an event"),
            })
            .await;
            next.await
        });

        waiting_rx.await.unwrap();
        buffer.push(7);

        assert!(matches!(consumer.await.unwrap(), BufferEvent::Item(7)));
    }
}
