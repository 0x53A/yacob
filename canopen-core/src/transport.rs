use heapless::Deque;

/// A concrete CAN frame for CANopen (11-bit standard ID, up to 8 data bytes).
///
/// This is the library's internal frame type. Convert to/from `embedded_can::Frame`
/// implementations using `from_frame()` and `to_frame()`.
#[derive(Clone, Copy, Debug)]
pub struct CanFrame {
    id: u16,
    dlc: u8,
    data: [u8; 8],
}

impl CanFrame {
    /// Create a new data frame. Returns None if id > 0x7FF or data > 8 bytes.
    pub const fn new(id: u16, data: &[u8]) -> Option<Self> {
        if id > 0x7FF || data.len() > 8 {
            return None;
        }
        let mut frame_data = [0u8; 8];
        let mut i = 0;
        while i < data.len() {
            frame_data[i] = data[i];
            i += 1;
        }
        Some(Self {
            id,
            dlc: data.len() as u8,
            data: frame_data,
        })
    }

    pub const fn id(&self) -> u16 {
        self.id
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }

    pub const fn dlc(&self) -> u8 {
        self.dlc
    }

    /// Convert from any `embedded_can::Frame`. Returns None for extended IDs.
    pub fn from_frame<F: embedded_can::Frame>(frame: &F) -> Option<Self> {
        match frame.id() {
            embedded_can::Id::Standard(sid) => {
                let mut data = [0u8; 8];
                let d = frame.data();
                data[..d.len()].copy_from_slice(d);
                Some(Self {
                    id: sid.as_raw(),
                    dlc: d.len() as u8,
                    data,
                })
            }
            embedded_can::Id::Extended(_) => None,
        }
    }

    /// Convert to any `embedded_can::Frame` type.
    pub fn to_frame<F: embedded_can::Frame>(&self) -> Option<F> {
        let sid = embedded_can::StandardId::new(self.id)?;
        F::new(embedded_can::Id::Standard(sid), self.data())
    }
}

/// Error from transport operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransportError {
    TxBufferFull,
    BusError,
}

/// Polling-based CAN transport trait.
pub trait Transport {
    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError>;
    fn recv(&mut self) -> Option<CanFrame>;
}

/// A static mailbox transport using heapless queues.
///
/// Use this to bridge between interrupt-driven or async CAN drivers and the
/// polling-based `Node::process()`. The CAN driver side calls `store_received()`
/// and `next_to_transmit()`, while the protocol side uses the `Transport` trait.
pub struct MailboxTransport<const TX: usize = 16, const RX: usize = 16> {
    tx_queue: Deque<CanFrame, TX>,
    rx_queue: Deque<CanFrame, RX>,
}

impl<const TX: usize, const RX: usize> MailboxTransport<TX, RX> {
    pub const fn new() -> Self {
        Self {
            tx_queue: Deque::new(),
            rx_queue: Deque::new(),
        }
    }

    /// Called by the CAN driver when a frame is received.
    /// Returns Err with the frame if the rx buffer is full.
    pub fn store_received(&mut self, frame: CanFrame) -> Result<(), CanFrame> {
        self.rx_queue.push_back(frame)
    }

    /// Called by the CAN driver to get the next frame to transmit.
    pub fn next_to_transmit(&mut self) -> Option<CanFrame> {
        self.tx_queue.pop_front()
    }
}

impl<const TX: usize, const RX: usize> Transport for MailboxTransport<TX, RX> {
    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        self.tx_queue
            .push_back(*frame)
            .map_err(|_| TransportError::TxBufferFull)
    }

    fn recv(&mut self) -> Option<CanFrame> {
        self.rx_queue.pop_front()
    }
}

/// Adapter wrapping an `embedded_can::blocking::Can` into `Transport`.
pub struct BlockingTransport<C> {
    can: C,
}

impl<C> BlockingTransport<C> {
    pub fn new(can: C) -> Self {
        Self { can }
    }

    pub fn inner(&self) -> &C {
        &self.can
    }

    pub fn inner_mut(&mut self) -> &mut C {
        &mut self.can
    }
}

impl<C> Transport for BlockingTransport<C>
where
    C: embedded_can::blocking::Can,
{
    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let f: C::Frame = frame.to_frame().ok_or(TransportError::BusError)?;
        self.can.transmit(&f).map_err(|_| TransportError::BusError)
    }

    fn recv(&mut self) -> Option<CanFrame> {
        // blocking::Can::receive() blocks, so we can't use it in a polling context.
        // Users of blocking CAN should use MailboxTransport with interrupt-driven rx instead.
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canframe_basic() {
        let f = CanFrame::new(0x601, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap();
        assert_eq!(f.id(), 0x601);
        assert_eq!(f.dlc(), 8);
        assert_eq!(f.data(), &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]);
    }

    #[test]
    fn canframe_rejects_invalid() {
        assert!(CanFrame::new(0x800, &[]).is_none());
        assert!(CanFrame::new(0x100, &[0; 9]).is_none());
    }

    #[test]
    fn mailbox_transport() {
        let mut mbox = MailboxTransport::<4, 4>::new();
        let frame = CanFrame::new(0x701, &[0x05]).unwrap();

        // Send via Transport trait
        mbox.send(&frame).unwrap();
        // Retrieve from driver side
        let tx = mbox.next_to_transmit().unwrap();
        assert_eq!(tx.id(), 0x701);

        // Store from driver side
        mbox.store_received(frame).unwrap();
        // Receive via Transport trait
        let rx = mbox.recv().unwrap();
        assert_eq!(rx.id(), 0x701);
    }
}
