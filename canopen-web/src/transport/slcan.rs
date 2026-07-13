use super::{js_error, CanEvent, CanTransport};
use canopen_core::slcan::{encode_slcan_frame, parse_slcan_frame};
use canopen_core::transport::CanFrame;
use js_sys::Uint8Array;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::{spawn_local, JsFuture};
use web_sys::{
    ReadableStreamDefaultReader, SerialOptions, SerialPort, WritableStreamDefaultWriter,
};

pub struct SlcanTransport {
    inner: Rc<RefCell<Inner>>,
}

#[derive(Default)]
struct Inner {
    events: VecDeque<CanEvent>,
    running: bool,
    line_buf: Vec<u8>,
    serial_port: Option<SerialPort>,
    serial_writer: Option<WritableStreamDefaultWriter>,
}

impl SlcanTransport {
    pub fn connect() -> Self {
        let inner = Rc::new(RefCell::new(Inner::default()));
        connect(inner.clone());
        Self { inner }
    }
}

impl CanTransport for SlcanTransport {
    fn disconnect(&mut self) {
        disconnect_inner(self.inner.clone());
    }

    fn send(&self, frame: CanFrame) {
        let writer = self.inner.borrow().serial_writer.clone();
        if let Some(writer) = writer {
            spawn_local(async move {
                let mut line = [0u8; 32];
                if let Some(len) = encode_slcan_frame(&frame, &mut line) {
                    let _ = write_serial_bytes(&writer, &line[..len]).await;
                }
            });
        }
    }

    fn poll_event(&mut self) -> Option<CanEvent> {
        self.inner.borrow_mut().events.pop_front()
    }
}

impl Drop for SlcanTransport {
    fn drop(&mut self) {
        disconnect_inner(self.inner.clone());
    }
}

fn connect(inner: Rc<RefCell<Inner>>) {
    spawn_local(async move {
        if let Err(e) = connect_async(inner.clone()).await {
            let mut inner = inner.borrow_mut();
            inner.running = false;
            inner.events.push_back(CanEvent::Error(e));
        }
    });
}

async fn connect_async(inner: Rc<RefCell<Inner>>) -> Result<(), String> {
    let window = web_sys::window().ok_or("window unavailable")?;
    let serial = window.navigator().serial();
    let port = JsFuture::from(serial.request_port())
        .await
        .map_err(js_error)?
        .dyn_into::<SerialPort>()
        .map_err(|_| "requestPort did not return a SerialPort".to_string())?;

    JsFuture::from(port.open(&SerialOptions::new(115_200)))
        .await
        .map_err(js_error)?;

    let writer = WritableStreamDefaultWriter::new(&port.writable()).map_err(js_error)?;
    write_serial_bytes(&writer, b"C\r").await.ok();
    write_serial_bytes(&writer, b"S6\r").await?;
    write_serial_bytes(&writer, b"O\r").await?;

    let reader = ReadableStreamDefaultReader::new(&port.readable()).map_err(js_error)?;
    {
        let mut inner = inner.borrow_mut();
        inner.running = true;
        inner.serial_port = Some(port);
        inner.serial_writer = Some(writer);
        inner.events.push_back(CanEvent::Connected);
    }

    read_loop(inner, reader).await;
    Ok(())
}

async fn read_loop(inner: Rc<RefCell<Inner>>, reader: ReadableStreamDefaultReader) {
    while inner.borrow().running {
        let result = match JsFuture::from(reader.read()).await {
            Ok(result) => result,
            Err(e) => {
                inner
                    .borrow_mut()
                    .events
                    .push_back(CanEvent::Error(js_error(e)));
                break;
            }
        };

        let done = js_sys::Reflect::get(&result, &JsValue::from_str("done"))
            .ok()
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        if done {
            break;
        }

        let Ok(value) = js_sys::Reflect::get(&result, &JsValue::from_str("value")) else {
            continue;
        };
        if value.is_undefined() {
            continue;
        }
        let bytes = Uint8Array::new(&value).to_vec();
        for byte in bytes {
            process_byte(&inner, byte);
        }
    }

    let mut inner = inner.borrow_mut();
    inner.running = false;
    inner.events.push_back(CanEvent::Disconnected);
}

fn process_byte(inner: &Rc<RefCell<Inner>>, byte: u8) {
    if byte == b'\r' || byte == b'\n' {
        let line = {
            let inner_ref = inner.borrow();
            if inner_ref.line_buf.is_empty() {
                return;
            }
            String::from_utf8_lossy(&inner_ref.line_buf).to_string()
        };
        inner.borrow_mut().line_buf.clear();
        if let Some(frame) = parse_slcan_frame(line.as_bytes()) {
            inner.borrow_mut().events.push_back(CanEvent::Frame(frame));
        }
    } else {
        let mut inner = inner.borrow_mut();
        if inner.line_buf.len() < 64 {
            inner.line_buf.push(byte);
        } else {
            inner.line_buf.clear();
        }
    }
}

fn disconnect_inner(inner: Rc<RefCell<Inner>>) {
    let (port, writer) = {
        let mut inner = inner.borrow_mut();
        inner.running = false;
        (inner.serial_port.take(), inner.serial_writer.take())
    };

    if let Some(writer) = writer {
        spawn_local(async move {
            let _ = write_serial_bytes(&writer, b"C\r").await;
            let _ = JsFuture::from(writer.close()).await;
        });
    }

    if let Some(port) = port {
        spawn_local(async move {
            let _ = JsFuture::from(port.close()).await;
        });
    }
}

async fn write_serial_bytes(
    writer: &WritableStreamDefaultWriter,
    bytes: &[u8],
) -> Result<(), String> {
    let array = Uint8Array::from(bytes);
    JsFuture::from(writer.write_with_chunk(&array))
        .await
        .map(|_| ())
        .map_err(js_error)
}
