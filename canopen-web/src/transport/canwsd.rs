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
    inner: Rc<RefCell<Inner>>,
}

#[derive(Default)]
struct Inner {
    events: VecDeque<CanEvent>,
    running: bool,
    ws: Option<WebSocket>,
    ws_onopen: Option<Closure<dyn FnMut(WebEvent)>>,
    ws_onmessage: Option<Closure<dyn FnMut(MessageEvent)>>,
    ws_onerror: Option<Closure<dyn FnMut(ErrorEvent)>>,
    ws_onclose: Option<Closure<dyn FnMut(WebEvent)>>,
}

impl CanwsdTransport {
    pub fn connect(url: impl AsRef<str>) -> Self {
        let inner = Rc::new(RefCell::new(Inner::default()));
        connect(inner.clone(), url.as_ref().to_string());
        Self { inner }
    }
}

impl CanTransport for CanwsdTransport {
    fn disconnect(&mut self) {
        disconnect_inner(self.inner.clone());
    }

    fn send(&self, frame: CanFrame) {
        let Some(ws) = self.inner.borrow().ws.clone() else {
            return;
        };
        if let Some(wire) = WireFrame::new(frame.raw_id() as u32, frame.data()) {
            let (buf, len) = wire.encode();
            let _ = ws.send_with_u8_array(&buf[..len]);
        }
    }

    fn poll_event(&mut self) -> Option<CanEvent> {
        self.inner.borrow_mut().events.pop_front()
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

fn connect(inner: Rc<RefCell<Inner>>, url: String) {
    let ws = match WebSocket::new(&url) {
        Ok(ws) => ws,
        Err(e) => {
            inner
                .borrow_mut()
                .events
                .push_back(CanEvent::Error(js_error(e)));
            return;
        }
    };
    ws.set_binary_type(BinaryType::Arraybuffer);

    let onopen_inner = inner.clone();
    let onopen = Closure::wrap(Box::new(move |_event: WebEvent| {
        let mut inner = onopen_inner.borrow_mut();
        inner.running = true;
        inner.events.push_back(CanEvent::Connected);
    }) as Box<dyn FnMut(_)>);
    ws.set_onopen(Some(onopen.as_ref().unchecked_ref()));

    let onmessage_inner = inner.clone();
    let onmessage = Closure::wrap(Box::new(move |event: MessageEvent| {
        if let Ok(array) = event.data().dyn_into::<ArrayBuffer>() {
            let data = Uint8Array::new(&array).to_vec();
            if let Ok(frame) = decode_wire_frame(&data) {
                onmessage_inner
                    .borrow_mut()
                    .events
                    .push_back(CanEvent::Frame(frame));
            }
        }
    }) as Box<dyn FnMut(_)>);
    ws.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));

    let onerror_inner = inner.clone();
    let onerror = Closure::wrap(Box::new(move |event: ErrorEvent| {
        onerror_inner
            .borrow_mut()
            .events
            .push_back(CanEvent::Error(event.message()));
    }) as Box<dyn FnMut(_)>);
    ws.set_onerror(Some(onerror.as_ref().unchecked_ref()));

    let onclose_inner = inner.clone();
    let onclose = Closure::wrap(Box::new(move |_event: WebEvent| {
        let mut inner = onclose_inner.borrow_mut();
        inner.running = false;
        inner.events.push_back(CanEvent::Disconnected);
    }) as Box<dyn FnMut(_)>);
    ws.set_onclose(Some(onclose.as_ref().unchecked_ref()));

    let mut state = inner.borrow_mut();
    state.ws = Some(ws);
    state.ws_onopen = Some(onopen);
    state.ws_onmessage = Some(onmessage);
    state.ws_onerror = Some(onerror);
    state.ws_onclose = Some(onclose);
}

fn disconnect_inner(inner: Rc<RefCell<Inner>>) {
    let ws = {
        let mut inner = inner.borrow_mut();
        inner.running = false;
        inner.ws.take()
    };

    if let Some(ws) = ws {
        let _ = ws.close();
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
