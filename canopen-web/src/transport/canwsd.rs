use super::{js_error, CanEvent, CanTransport};
use canopen_core::transport::CanFrame;
use canwsd_proto::wire::{CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_RTR_FLAG};
use canwsd_proto::{NetworkInfo, WireFrame, NETWORKS_PATH};
use js_sys::{ArrayBuffer, Uint8Array};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;
use web_sys::{BinaryType, ErrorEvent, Event as WebEvent, MessageEvent, Response, WebSocket};

pub struct CanwsdTransport {
    inner: Rc<Inner>,
}

/// Two disjoint cells so the frame/event queue and the WebSocket handle never
/// share a borrow. The browser callbacks and `poll_event` touch only `events`;
/// `send`/`connect`/`disconnect` touch only `session` (`connect` also seeds
/// `events` on its constructor-error path, but that runs outside any callback).
#[derive(Default)]
struct Inner {
    events: RefCell<VecDeque<CanEvent>>,
    session: RefCell<Session>,
}

#[derive(Default)]
struct Session {
    ws: Option<WebSocket>,
    onopen: Option<Closure<dyn FnMut(WebEvent)>>,
    onmessage: Option<Closure<dyn FnMut(MessageEvent)>>,
    onerror: Option<Closure<dyn FnMut(ErrorEvent)>>,
    onclose: Option<Closure<dyn FnMut(WebEvent)>>,
}

impl CanwsdTransport {
    pub fn connect(url: impl AsRef<str>) -> Self {
        let inner = Rc::new(Inner::default());
        connect(inner.clone(), url.as_ref().to_string());
        Self { inner }
    }
}

// Borrows are held only for the duration of a single field access and never
// across a call into JS, and the browser never dispatches these callbacks
// re-entrantly, so the `events` and `session` borrows never nest. Plain
// `borrow`/`borrow_mut` is used deliberately: if that invariant is ever
// violated it panics loudly rather than silently dropping a frame or event.
impl CanTransport for CanwsdTransport {
    fn disconnect(&mut self) {
        disconnect_inner(self.inner.clone());
    }

    fn send(&self, frame: CanFrame) {
        let Some(ws) = self.inner.session.borrow().ws.clone() else {
            return;
        };
        if let Some(wire) = WireFrame::new(frame.raw_id() as u32, frame.data()) {
            let (buf, len) = wire.encode();
            let _ = ws.send_with_u8_array(&buf[..len]);
        }
    }

    fn poll_event(&mut self) -> Option<CanEvent> {
        self.inner.events.borrow_mut().pop_front()
    }
}

impl Drop for CanwsdTransport {
    fn drop(&mut self) {
        disconnect_inner(self.inner.clone());
    }
}

pub async fn fetch_canwsd_networks(base_url: &str) -> Result<Vec<String>, String> {
    let url = format!("{}{NETWORKS_PATH}", base_url.trim_end_matches('/'));
    let window = web_sys::window().ok_or("window unavailable")?;
    let response = JsFuture::from(window.fetch_with_str(&url))
        .await
        .map_err(js_error)?
        .dyn_into::<Response>()
        .map_err(|_| "fetch did not return a Response".to_string())?;
    if !response.ok() {
        return Err(format!("HTTP {}", response.status()));
    }
    let json = JsFuture::from(response.json().map_err(js_error)?)
        .await
        .map_err(js_error)?;
    let networks: Vec<NetworkInfo> =
        serde_wasm_bindgen::from_value(json).map_err(|e| e.to_string())?;
    Ok(networks.into_iter().map(|n| n.name).collect())
}

fn connect(inner: Rc<Inner>, url: String) {
    let ws = match WebSocket::new(&url) {
        Ok(ws) => ws,
        Err(e) => {
            inner
                .events
                .borrow_mut()
                .push_back(CanEvent::Error(js_error(e)));
            return;
        }
    };
    ws.set_binary_type(BinaryType::Arraybuffer);

    let onopen_inner = inner.clone();
    let onopen = Closure::wrap(Box::new(move |_event: WebEvent| {
        onopen_inner
            .events
            .borrow_mut()
            .push_back(CanEvent::Connected);
    }) as Box<dyn FnMut(_)>);
    ws.set_onopen(Some(onopen.as_ref().unchecked_ref()));

    let onmessage_inner = inner.clone();
    let onmessage = Closure::wrap(Box::new(move |event: MessageEvent| {
        if let Ok(array) = event.data().dyn_into::<ArrayBuffer>() {
            let data = Uint8Array::new(&array).to_vec();
            if let Ok(frame) = decode_wire_frame(&data) {
                onmessage_inner
                    .events
                    .borrow_mut()
                    .push_back(CanEvent::Frame(frame));
            }
        }
    }) as Box<dyn FnMut(_)>);
    ws.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));

    let onerror_inner = inner.clone();
    let onerror = Closure::wrap(Box::new(move |event: ErrorEvent| {
        onerror_inner
            .events
            .borrow_mut()
            .push_back(CanEvent::Error(error_message(event)));
    }) as Box<dyn FnMut(_)>);
    ws.set_onerror(Some(onerror.as_ref().unchecked_ref()));

    let onclose_inner = inner.clone();
    let onclose = Closure::wrap(Box::new(move |_event: WebEvent| {
        onclose_inner
            .events
            .borrow_mut()
            .push_back(CanEvent::Disconnected);
    }) as Box<dyn FnMut(_)>);
    ws.set_onclose(Some(onclose.as_ref().unchecked_ref()));

    let mut session = inner.session.borrow_mut();
    session.ws = Some(ws);
    session.onopen = Some(onopen);
    session.onmessage = Some(onmessage);
    session.onerror = Some(onerror);
    session.onclose = Some(onclose);
}

fn disconnect_inner(inner: Rc<Inner>) {
    let ws = { inner.session.borrow().ws.clone() };
    if let Some(ws) = ws.as_ref() {
        ws.set_onopen(None);
        ws.set_onmessage(None);
        ws.set_onerror(None);
        ws.set_onclose(None);
    }

    let ws = {
        let mut session = inner.session.borrow_mut();
        session.onopen = None;
        session.onmessage = None;
        session.onerror = None;
        session.onclose = None;
        session.ws.take()
    };
    if let Some(ws) = ws {
        let _ = ws.close();
    }
}

fn error_message(event: ErrorEvent) -> String {
    let message = js_error(event.into());
    if message.is_empty() {
        "WebSocket error".to_string()
    } else {
        message
    }
}

/// Decode one WebSocket message into a CANopen frame. Frames with EFF/RTR/ERR
/// flags are valid on the wire but have no meaning for CANopen and are
/// rejected here.
fn decode_wire_frame(buf: &[u8]) -> Result<CanFrame, ()> {
    let wire = WireFrame::decode(buf).map_err(|_| ())?;
    if wire.id_word() & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG) != 0 {
        return Err(());
    }
    CanFrame::new(wire.id_word() as u16, wire.data()).ok_or(())
}
