//! High-level SDO client helpers for Linux.
//!
//! These wrap the async `SdoDriver` into blocking read/write calls with timeouts,
//! suitable for test harnesses and CLI tools.

use canopen_core::can_router::SdoPort;
use canopen_core::cobid::{CobId, NodeId};
use canopen_core::sdo::driver::{SdoDriver, SdoError as AsyncSdoError};
use canopen_core::sdo::AbortCode;
use canopen_core::transport::CanFrame;
use std::time::{Duration, Instant};

/// Error from a high-level SDO operation.
#[derive(Debug)]
pub enum SdoError {
    Aborted(AbortCode),
    Timeout,
    ProtocolError,
    TransportError,
    /// Download data exceeds the SDO client's transfer buffer.
    TooLarge,
}

impl std::fmt::Display for SdoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Aborted(code) => write!(f, "SDO aborted: {:?}", code),
            Self::Timeout => write!(f, "SDO timeout"),
            Self::ProtocolError => write!(f, "SDO protocol error"),
            Self::TransportError => write!(f, "CAN transport error"),
            Self::TooLarge => write!(f, "SDO download data exceeds client buffer"),
        }
    }
}

impl std::error::Error for SdoError {}

impl<E: core::fmt::Debug> From<AsyncSdoError<E>> for SdoError {
    fn from(e: AsyncSdoError<E>) -> Self {
        match e {
            AsyncSdoError::Aborted(code) => Self::Aborted(code),
            AsyncSdoError::ProtocolError => Self::ProtocolError,
            AsyncSdoError::Transport(_) => Self::TransportError,
            AsyncSdoError::Timeout => Self::Timeout,
            AsyncSdoError::TooLarge => Self::TooLarge,
        }
    }
}

/// Minimal block_on executor with timeout for running async SDO operations.
///
/// Polls the future in a 100µs sleep loop; returns `SdoError::Timeout` if it
/// doesn't complete within `timeout`. Public so applications can drive
/// `SdoDriver`/generated EDS clients over their own `AsyncCan` ports (e.g. a
/// `CanDemux` sdo_port) with a hard deadline even on a silent bus.
pub fn block_on_with_timeout<F: core::future::Future>(
    f: F,
    timeout: Duration,
) -> Result<F::Output, SdoError> {
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
    let mut f = pin!(f);
    let deadline = Instant::now() + timeout;

    loop {
        match f.as_mut().poll(&mut cx) {
            Poll::Ready(val) => return Ok(val),
            Poll::Pending => {
                if Instant::now() > deadline {
                    return Err(SdoError::Timeout);
                }
                std::thread::sleep(Duration::from_micros(100));
            }
        }
    }
}

/// Perform a blocking SDO upload (read) from a remote node.
///
/// Returns the data read from the object dictionary entry.
pub fn sdo_upload<T>(
    transport: &mut T,
    target: NodeId,
    index: u16,
    subindex: u8,
    timeout: Duration,
) -> Result<Vec<u8>, SdoError>
where
    T: embedded_can::nb::Can<Frame = CanFrame>,
    T::Error: core::fmt::Debug,
{
    let driver = SdoDriver::new(target);
    let mut wrapper = NbCanWrapper(transport);
    let mut port = SdoPort::new(&mut wrapper, target);
    let mut buf = vec![0u8; 889]; // max SDO transfer

    let len = block_on_with_timeout(driver.upload(index, subindex, &mut buf, &mut port), timeout)??;
    buf.truncate(len);
    Ok(buf)
}

/// Perform a blocking SDO download (write) to a remote node.
pub fn sdo_download<T>(
    transport: &mut T,
    target: NodeId,
    index: u16,
    subindex: u8,
    data: &[u8],
    timeout: Duration,
) -> Result<(), SdoError>
where
    T: embedded_can::nb::Can<Frame = CanFrame>,
    T::Error: core::fmt::Debug,
{
    let driver = SdoDriver::new(target);
    let mut wrapper = NbCanWrapper(transport);
    let mut port = SdoPort::new(&mut wrapper, target);

    block_on_with_timeout(driver.download(index, subindex, data, &mut port), timeout)??;
    Ok(())
}

/// Wrapper that implements AsyncCan for &mut T where T: nb::Can.
/// Similar to NbCanAsync but works with mutable references.
struct NbCanWrapper<'a, T>(&'a mut T);

impl<T> canopen_core::sdo::AsyncCan for NbCanWrapper<'_, T>
where
    T: embedded_can::nb::Can<Frame = CanFrame>,
    T::Error: core::fmt::Debug,
{
    type Error = T::Error;

    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        loop {
            match self.0.transmit(frame) {
                Ok(_) => return Ok(()),
                Err(nb::Error::WouldBlock) => {
                    // Yield then retry
                    let mut yielded = false;
                    core::future::poll_fn(|cx| {
                        if yielded {
                            core::task::Poll::Ready(())
                        } else {
                            yielded = true;
                            cx.waker().wake_by_ref();
                            core::task::Poll::Pending
                        }
                    })
                    .await;
                }
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }
    }

    async fn receive(&mut self) -> Result<CanFrame, Self::Error> {
        loop {
            match self.0.receive() {
                Ok(frame) => return Ok(frame),
                Err(nb::Error::WouldBlock) => {
                    let mut yielded = false;
                    core::future::poll_fn(|cx| {
                        if yielded {
                            core::task::Poll::Ready(())
                        } else {
                            yielded = true;
                            cx.waker().wake_by_ref();
                            core::task::Poll::Pending
                        }
                    })
                    .await;
                }
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }
    }
}

fn send(
    transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
    frame: &CanFrame,
) -> Result<(), SdoError> {
    transport
        .transmit(frame)
        .map_err(|_| SdoError::TransportError)?;
    Ok(())
}

fn try_recv(transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>) -> Option<CanFrame> {
    transport.receive().ok()
}

/// Send an NMT command to a node (or broadcast with node_id=0).
pub fn nmt_command(
    transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
    command: u8,
    target_node: u8,
) -> Result<(), SdoError> {
    let frame = CanFrame::new(0x000, &[command, target_node]).unwrap();
    send(transport, &frame)
}

/// Wait for a heartbeat from a specific node. Returns the NMT state byte.
pub fn wait_heartbeat(
    transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
    target: NodeId,
    timeout: Duration,
) -> Result<u8, SdoError> {
    let hb_cob = CobId::heartbeat(target).raw();
    let deadline = Instant::now() + timeout;

    loop {
        if Instant::now() > deadline {
            return Err(SdoError::Timeout);
        }
        if let Some(frame) = try_recv(transport) {
            if frame.raw_id() == hb_cob && frame.raw_dlc() >= 1 {
                return Ok(frame.data()[0]);
            }
        }
        std::thread::sleep(Duration::from_micros(100));
    }
}
