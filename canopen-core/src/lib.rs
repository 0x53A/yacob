#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;

pub use heapless;

pub mod can_router;
pub mod cobid;
#[cfg(feature = "alloc")]
pub mod dcf;
pub mod datatypes;
pub mod emcy;
pub mod heartbeat;
pub mod lss;
pub mod nmt;
pub mod node;
pub mod od;
pub mod pdo;
pub mod sdo;
pub mod sync;
pub mod time;
pub mod transport;

pub use cobid::{CobId, NodeId, ParsedCobId};
pub use datatypes::DataType;
pub use emcy::{EmcyErrorCode, EmcyProducer, build_emcy_frame, error_register};
pub use lss::{LssEvent, LssIdentity, LssMode, LssSlave};
pub use nmt::{NmtCommand, NmtHandler, NmtState, NmtTransition};
pub use node::{Node, ResetType};
pub use od::{AccessType, ObjectDictionary, ObjectType, OdEntryMeta, OdError, OdEvent, OdEventSource};
#[cfg(feature = "embassy")]
pub use od::OdEventSignal;
pub use time::Clock;
pub use pdo::engine::{
    sync_cyclic, EVENT_DRIVEN, EVENT_DRIVEN_MANUFACTURER, SYNC_ACYCLIC, SYNC_CYCLIC_1,
};
pub use transport::{CanError, CanFrame, MailboxTransport};
