//! Device-specific panel seam.
//!
//! The monitor knows about CANopen in general — node discovery, NMT, frames,
//! raw SDO. Anything that depends on a *device profile* (CiA 402 drives, a
//! drilling unit, …) is a [`DevicePanel`] supplied by the embedding app, so
//! profile knowledge never leaks into this crate.

use crate::app::NodeInfo;
use crate::can_bus::Command;
use crate::sdo::{SdoOutcome, SdoRequest};
use eframe::egui;
use std::collections::{BTreeMap, VecDeque};

pub type SdoRequestId = u64;

/// What a [`DevicePanel`] may see and do. Commands are collected here rather
/// than sent directly so the panel can borrow node state at the same time.
pub struct BusCtx<'a> {
    pub selected_node: Option<u8>,
    pub connected: bool,
    pub nodes: &'a BTreeMap<u8, NodeInfo>,
    commands: &'a mut Vec<Command>,
    results: &'a mut VecDeque<SdoOutcome>,
    next_request_id: &'a mut SdoRequestId,
}

impl<'a> BusCtx<'a> {
    pub(crate) fn new(
        selected_node: Option<u8>,
        connected: bool,
        nodes: &'a BTreeMap<u8, NodeInfo>,
        commands: &'a mut Vec<Command>,
        results: &'a mut VecDeque<SdoOutcome>,
        next_request_id: &'a mut SdoRequestId,
    ) -> Self {
        Self {
            selected_node,
            connected,
            nodes,
            commands,
            results,
            next_request_id,
        }
    }

    pub fn selected(&self) -> Option<(u8, &NodeInfo)> {
        let id = self.selected_node?;
        Some((id, self.nodes.get(&id)?))
    }

    fn request_id(&mut self) -> SdoRequestId {
        let id = *self.next_request_id;
        *self.next_request_id = self.next_request_id.wrapping_add(1).max(1);
        id
    }

    pub fn sdo_read(&mut self, node_id: u8, index: u16, subindex: u8) -> SdoRequestId {
        let request_id = self.request_id();
        self.commands.push(Command::Sdo {
            request: SdoRequest::read(request_id, node_id, index, subindex),
            priority: false,
        });
        request_id
    }

    pub fn sdo_write(
        &mut self,
        node_id: u8,
        index: u16,
        subindex: u8,
        data: Vec<u8>,
    ) -> SdoRequestId {
        self.sdo_write_inner(node_id, index, subindex, data, false)
    }

    pub fn sdo_write_priority(
        &mut self,
        node_id: u8,
        index: u16,
        subindex: u8,
        data: Vec<u8>,
    ) -> SdoRequestId {
        self.sdo_write_inner(node_id, index, subindex, data, true)
    }

    fn sdo_write_inner(
        &mut self,
        node_id: u8,
        index: u16,
        subindex: u8,
        data: Vec<u8>,
        priority: bool,
    ) -> SdoRequestId {
        let request_id = self.request_id();
        self.commands.push(Command::Sdo {
            request: SdoRequest::write(request_id, node_id, index, subindex, data),
            priority,
        });
        request_id
    }

    pub fn cancel_sdo(&mut self, node_id: u8) {
        self.commands.push(Command::CancelSdo { node_id });
    }

    pub fn nmt(&mut self, node_id: u8, command: u8) {
        self.commands.push(Command::Nmt { node_id, command });
    }

    /// Drain one completed SDO transfer. Results are delivered to the panel
    /// only — the monitor itself does not interpret them.
    pub fn next_result(&mut self) -> Option<SdoOutcome> {
        self.results.pop_front()
    }
}

pub trait DevicePanel {
    /// Title shown above the panel.
    fn title(&self) -> &str;

    /// Called every frame with the panel's own region.
    fn ui(&mut self, ui: &mut egui::Ui, bus: &mut BusCtx<'_>);

    /// Desired initial height of the panel region, in points.
    fn preferred_height(&self) -> f32 {
        320.0
    }
}

/// No device panel — the standalone monitor. `App<NoPanel>` is the plain
/// `canopen-egui` binary.
#[derive(Default)]
pub struct NoPanel;

impl DevicePanel for NoPanel {
    fn title(&self) -> &str {
        ""
    }

    fn ui(&mut self, _ui: &mut egui::Ui, _bus: &mut BusCtx<'_>) {}

    fn preferred_height(&self) -> f32 {
        0.0
    }
}
