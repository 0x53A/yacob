use socketcan::{CanFrame, CanRemoteFrame, EmbeddedFrame, ExtendedId, Frame, Id, StandardId};

use canwsd_proto::WireFrame;

pub fn wire_from_can_frame(frame: &CanFrame) -> Option<WireFrame> {
    // RTR frames: socketcan's data() yields DLC zero bytes, so the wire frame
    // keeps the DLC.
    WireFrame::new(frame.id_word(), frame.data())
}

/// Build a socketcan frame from a decoded wire frame. `None` for frames that
/// cannot be transmitted (error frames, out-of-range IDs).
///
/// Not `CanFrame::from_raw_id`: that feeds the whole id word (flags included)
/// into the ID range check, so every EFF or RTR frame would be rejected.
pub fn can_frame_from_wire(wf: &WireFrame) -> Option<CanFrame> {
    if wf.is_error() {
        return None;
    }
    let id: Id = if wf.is_extended() {
        ExtendedId::new(wf.id())?.into()
    } else {
        StandardId::new(wf.id() as u16)?.into()
    };
    if wf.is_rtr() {
        CanRemoteFrame::new_remote(id, wf.dlc() as usize).map(CanFrame::Remote)
    } else {
        CanFrame::new(id, wf.data())
    }
}
