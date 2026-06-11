use core::cell::RefCell;
use critical_section::Mutex;
use heapless::Deque;

/// CAN error type for canopen-rs transports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanError {
    TxBufferFull,
    BusError,
}

impl embedded_can::Error for CanError {
    fn kind(&self) -> embedded_can::ErrorKind {
        match self {
            CanError::TxBufferFull => embedded_can::ErrorKind::Overrun,
            CanError::BusError => embedded_can::ErrorKind::Other,
        }
    }
}

/// A concrete CAN frame for CANopen (11-bit standard ID, up to 8 data bytes).
///
/// Implements `embedded_can::Frame` for interoperability with any CAN driver.
/// Also provides inherent methods that return the more convenient raw types
/// (u16 id, u8 dlc) used internally throughout the protocol stack.
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

    /// Raw 11-bit standard CAN ID.
    pub const fn raw_id(&self) -> u16 {
        self.id
    }

    /// Frame data (0..8 bytes).
    pub fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }

    /// Data length code as u8.
    pub const fn raw_dlc(&self) -> u8 {
        self.dlc
    }
}

impl embedded_can::Frame for CanFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        match id.into() {
            embedded_can::Id::Standard(sid) => Self::new(sid.as_raw(), data),
            embedded_can::Id::Extended(_) => None, // CANopen uses standard IDs only
        }
    }

    fn new_remote(_id: impl Into<embedded_can::Id>, _dlc: usize) -> Option<Self> {
        None // CANopen does not use remote frames
    }

    fn is_extended(&self) -> bool {
        false
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> embedded_can::Id {
        // Safety: self.id is always <= 0x7FF by construction
        embedded_can::Id::Standard(embedded_can::StandardId::new(self.id).unwrap())
    }

    fn dlc(&self) -> usize {
        self.dlc as usize
    }

    fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

/// A `Send + Sync` mailbox transport using `critical_section`-protected queues.
///
/// Bridges interrupt-driven or async CAN drivers to the polling-based
/// `Node::process()`. The driver side calls `store_received()` and
/// `next_to_transmit()` with only `&self` — safe to call from ISR context.
/// The protocol side uses `embedded_can::nb::Can` (which takes `&mut self`,
/// held by the `Node` owner).
pub struct MailboxTransport<const TX: usize = 16, const RX: usize = 16> {
    tx_queue: Mutex<RefCell<Deque<CanFrame, TX>>>,
    rx_queue: Mutex<RefCell<Deque<CanFrame, RX>>>,
}

// Safety: all access goes through critical_section::Mutex
unsafe impl<const TX: usize, const RX: usize> Send for MailboxTransport<TX, RX> {}
unsafe impl<const TX: usize, const RX: usize> Sync for MailboxTransport<TX, RX> {}

impl<const TX: usize, const RX: usize> MailboxTransport<TX, RX> {
    pub const fn new() -> Self {
        Self {
            tx_queue: Mutex::new(RefCell::new(Deque::new())),
            rx_queue: Mutex::new(RefCell::new(Deque::new())),
        }
    }

    /// Called by the CAN driver when a frame is received.
    /// Safe to call from ISR context (`&self`, not `&mut self`).
    /// Returns Err with the frame if the rx buffer is full.
    pub fn store_received(&self, frame: CanFrame) -> Result<(), CanFrame> {
        critical_section::with(|cs| self.rx_queue.borrow(cs).borrow_mut().push_back(frame))
    }

    /// Called by the CAN driver to get the next frame to transmit.
    /// Safe to call from ISR context (`&self`, not `&mut self`).
    pub fn next_to_transmit(&self) -> Option<CanFrame> {
        critical_section::with(|cs| self.tx_queue.borrow(cs).borrow_mut().pop_front())
    }
}

impl<const TX: usize, const RX: usize> embedded_can::nb::Can for MailboxTransport<TX, RX> {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        critical_section::with(|cs| {
            self.tx_queue
                .borrow(cs)
                .borrow_mut()
                .push_back(*frame)
                .map_err(|_| nb::Error::Other(CanError::TxBufferFull))
        })?;
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        critical_section::with(|cs| {
            self.rx_queue
                .borrow(cs)
                .borrow_mut()
                .pop_front()
                .ok_or(nb::Error::WouldBlock)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::nb::Can;

    #[test]
    fn canframe_basic() {
        let f = CanFrame::new(0x601, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap();
        assert_eq!(f.raw_id(), 0x601);
        assert_eq!(f.raw_dlc(), 8);
        assert_eq!(f.data(), &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]);
    }

    #[test]
    fn canframe_rejects_invalid() {
        assert!(CanFrame::new(0x800, &[]).is_none());
        assert!(CanFrame::new(0x100, &[0; 9]).is_none());
    }

    #[test]
    fn canframe_embedded_can_trait() {
        use embedded_can::Frame;
        let sid = embedded_can::StandardId::new(0x701).unwrap();
        let f = <CanFrame as Frame>::new(sid, &[0x05]).unwrap();
        assert_eq!(f.raw_id(), 0x701);
        assert!(!f.is_extended());
        assert!(f.is_data_frame());
    }

    #[test]
    fn canframe_rejects_extended() {
        use embedded_can::Frame;
        let eid = embedded_can::ExtendedId::new(0x1234).unwrap();
        assert!(<CanFrame as Frame>::new(eid, &[]).is_none());
    }

    #[test]
    fn mailbox_transport() {
        let mut mbox = MailboxTransport::<4, 4>::new();
        let frame = CanFrame::new(0x701, &[0x05]).unwrap();

        // Send via nb::Can trait
        mbox.transmit(&frame).unwrap();
        // Retrieve from driver side
        let tx = mbox.next_to_transmit().unwrap();
        assert_eq!(tx.raw_id(), 0x701);

        // Store from driver side
        mbox.store_received(frame).unwrap();
        // Receive via nb::Can trait
        let rx = mbox.receive().unwrap();
        assert_eq!(rx.raw_id(), 0x701);

        // Empty receive returns WouldBlock
        assert!(matches!(mbox.receive(), Err(nb::Error::WouldBlock)));
    }
}
