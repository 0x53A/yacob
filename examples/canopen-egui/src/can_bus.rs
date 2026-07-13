use canopen_core::transport::CanFrame;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Backend {
    Slcan,
    Socketcan,
    Canwsd,
}

impl Backend {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Slcan => "SLCAN",
            Self::Socketcan => "SocketCAN",
            Self::Canwsd => "canwsd",
        }
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
pub enum ConnectionTarget {
    Slcan { device: String },
    Socketcan { interface: String },
    Canwsd { url: String },
}

#[derive(Debug)]
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
pub enum Command {
    Nmt { node_id: u8, command: u8 },
    Disconnect,
}

#[derive(Debug)]
pub enum Event {
    Connected,
    Disconnected,
    Error(String),
    Frame(CanFrame),
}

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::{Command, ConnectionTarget, Event};
    use canopen_core::transport::{CanError, CanFrame};
    use canopen_linux::slcan::{self, SlcanBitrate, SlcanTransport};
    use canopen_linux::{SocketcanTransport, WebSocketTransport};
    use embedded_can::nb::Can;
    use std::sync::mpsc;
    use std::time::Duration;

    pub struct CanConnection {
        cmd_tx: mpsc::Sender<Command>,
        event_rx: mpsc::Receiver<Event>,
        thread: Option<std::thread::JoinHandle<()>>,
    }

    impl CanConnection {
        pub fn connect(target: ConnectionTarget) -> Self {
            let (cmd_tx, cmd_rx) = mpsc::channel();
            let (event_tx, event_rx) = mpsc::channel();
            let thread = Some(start_can_thread(target, cmd_rx, event_tx));
            Self {
                cmd_tx,
                event_rx,
                thread,
            }
        }

        pub fn disconnect(&mut self) {
            let _ = self.cmd_tx.send(Command::Disconnect);
            if let Some(thread) = self.thread.take() {
                let _ = thread.join();
            }
        }

        pub fn send_cmd(&self, cmd: Command) {
            let _ = self.cmd_tx.send(cmd);
        }

        pub fn try_recv_event(&mut self) -> Option<Event> {
            self.event_rx.try_recv().ok()
        }
    }

    impl Drop for CanConnection {
        fn drop(&mut self) {
            self.disconnect();
        }
    }

    enum Transport {
        Slcan(SlcanTransport),
        Socketcan(SocketcanTransport),
        Canwsd(WebSocketTransport),
    }

    impl Transport {
        fn open(target: &ConnectionTarget) -> Result<Self, String> {
            match target {
                ConnectionTarget::Slcan { device } => {
                    slcan::open(device, SlcanBitrate::S6).map(Self::Slcan)
                }
                ConnectionTarget::Socketcan { interface } => SocketcanTransport::open(interface)
                    .map(Self::Socketcan)
                    .map_err(|e| e.to_string()),
                ConnectionTarget::Canwsd { url } => WebSocketTransport::connect(url)
                    .map(Self::Canwsd)
                    .map_err(|e| e.to_string()),
            }
        }

        fn close(&mut self) {
            if let Self::Slcan(transport) = self {
                let _ = transport.send_close();
            }
        }
    }

    impl Can for Transport {
        type Frame = CanFrame;
        type Error = CanError;

        fn transmit(
            &mut self,
            frame: &Self::Frame,
        ) -> nb::Result<Option<Self::Frame>, Self::Error> {
            match self {
                Self::Slcan(t) => t.transmit(frame),
                Self::Socketcan(t) => t.transmit(frame),
                Self::Canwsd(t) => t.transmit(frame),
            }
        }

        fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
            match self {
                Self::Slcan(t) => t.receive(),
                Self::Socketcan(t) => t.receive(),
                Self::Canwsd(t) => t.receive(),
            }
        }
    }

    fn start_can_thread(
        target: ConnectionTarget,
        cmd_rx: mpsc::Receiver<Command>,
        event_tx: mpsc::Sender<Event>,
    ) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            let mut transport = match Transport::open(&target) {
                Ok(transport) => transport,
                Err(e) => {
                    let _ = event_tx.send(Event::Error(e));
                    return;
                }
            };

            let _ = event_tx.send(Event::Connected);

            loop {
                match cmd_rx.try_recv() {
                    Ok(Command::Disconnect) => {
                        transport.close();
                        let _ = event_tx.send(Event::Disconnected);
                        return;
                    }
                    Ok(Command::Nmt { node_id, command }) => {
                        if let Some(frame) = CanFrame::new(0x000, &[command, node_id]) {
                            let _ = transport.transmit(&frame);
                        }
                    }
                    Err(mpsc::TryRecvError::Empty) => {}
                    Err(mpsc::TryRecvError::Disconnected) => return,
                }

                match transport.receive() {
                    Ok(frame) => {
                        let _ = event_tx.send(Event::Frame(frame));
                    }
                    Err(nb::Error::WouldBlock) => {
                        std::thread::sleep(Duration::from_micros(500));
                    }
                    Err(nb::Error::Other(_)) => {
                        std::thread::sleep(Duration::from_millis(10));
                    }
                }
            }
        })
    }
}

#[cfg(target_arch = "wasm32")]
mod web {
    use super::{Command, ConnectionTarget, Event};
    use canopen_core::transport::CanFrame;
    use canopen_web::transport::{
        canwsd::{self, CanwsdTransport},
        slcan::SlcanTransport,
        CanEvent, CanTransport,
    };
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use wasm_bindgen_futures::spawn_local;

    thread_local! {
        static NETWORK_RESULTS: RefCell<VecDeque<Result<Vec<String>, String>>> = RefCell::new(VecDeque::new());
    }

    pub struct CanConnection {
        transport: Option<Box<dyn CanTransport>>,
        local_events: VecDeque<Event>,
    }

    impl CanConnection {
        pub fn connect(target: ConnectionTarget) -> Self {
            let mut local_events = VecDeque::new();
            let transport = match target {
                ConnectionTarget::Slcan { .. } => {
                    Some(Box::new(SlcanTransport::connect()) as Box<dyn CanTransport>)
                }
                ConnectionTarget::Canwsd { url } => {
                    Some(Box::new(CanwsdTransport::connect(url)) as Box<dyn CanTransport>)
                }
                ConnectionTarget::Socketcan { .. } => {
                    local_events.push_back(Event::Error(
                        "SocketCAN is only available in native builds".into(),
                    ));
                    None
                }
            };
            Self {
                transport,
                local_events,
            }
        }

        pub fn disconnect(&mut self) {
            if let Some(transport) = &mut self.transport {
                transport.disconnect();
            }
            self.transport = None;
        }

        pub fn send_cmd(&self, cmd: Command) {
            match cmd {
                Command::Nmt { node_id, command } => {
                    if let Some(frame) = CanFrame::new(0x000, &[command, node_id]) {
                        if let Some(transport) = &self.transport {
                            transport.send(frame);
                        }
                    }
                }
                Command::Disconnect => {}
            }
        }

        pub fn try_recv_event(&mut self) -> Option<Event> {
            if let Some(event) = self.local_events.pop_front() {
                return Some(event);
            }

            self.transport
                .as_mut()
                .and_then(|transport| transport.poll_event())
                .map(|event| match event {
                    CanEvent::Connected => Event::Connected,
                    CanEvent::Disconnected => Event::Disconnected,
                    CanEvent::Error(e) => Event::Error(e),
                    CanEvent::Frame(frame) => Event::Frame(frame),
                })
        }
    }

    impl Drop for CanConnection {
        fn drop(&mut self) {
            self.disconnect();
        }
    }

    pub fn fetch_canwsd_networks_async(base_url: String) {
        spawn_local(async move {
            let result = canwsd::fetch_canwsd_networks(&base_url).await;
            NETWORK_RESULTS.with(|results| results.borrow_mut().push_back(result));
        });
    }

    pub fn try_recv_canwsd_networks() -> Option<Result<Vec<String>, String>> {
        NETWORK_RESULTS.with(|results| results.borrow_mut().pop_front())
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use native::CanConnection;

#[cfg(target_arch = "wasm32")]
pub use web::{fetch_canwsd_networks_async, try_recv_canwsd_networks, CanConnection};
