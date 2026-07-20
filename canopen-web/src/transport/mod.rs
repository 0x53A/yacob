use canopen_core::transport::CanFrame;
#[cfg(any(feature = "canwsd", feature = "slcan"))]
use wasm_bindgen::JsValue;

#[cfg(feature = "canwsd")]
pub mod canwsd;
#[cfg(feature = "slcan")]
pub mod slcan;

#[derive(Debug)]
pub enum CanEvent {
    Connected,
    Disconnected,
    Error(String),
    Frame(CanFrame),
}

pub trait CanTransport {
    fn disconnect(&mut self);
    fn send(&self, frame: CanFrame);
    fn poll_event(&mut self) -> Option<CanEvent>;
}

#[cfg(any(feature = "canwsd", feature = "slcan"))]
pub(crate) fn js_error(value: JsValue) -> String {
    value
        .as_string()
        .or_else(|| {
            js_sys::Reflect::get(&value, &JsValue::from_str("message"))
                .ok()
                .and_then(|v| v.as_string())
        })
        .unwrap_or_else(|| format!("{value:?}"))
}
