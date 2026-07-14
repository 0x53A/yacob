#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "std")]
extern crate std;

pub use heapless;

#[cfg(feature = "std")]
pub mod bus;
pub mod can_router;
pub mod cobid;
pub mod datatypes;
#[cfg(feature = "alloc")]
pub mod dcf;
pub mod emcy;
pub mod events;
pub mod heartbeat;
pub mod lss;
pub mod nmt;
pub mod node;
pub mod od;
pub mod pdo;
pub mod sdo;
pub mod slcan;
pub mod sync;
pub mod time;
pub mod transport;

#[cfg(feature = "std")]
pub use bus::{OverflowPolicy, SharedCanBus, Subscription, SubscriptionPort, TxQueue};
pub use cobid::{CobId, NodeId, ParsedCobId};
pub use datatypes::DataType;
pub use emcy::{build_emcy_frame, error_register, EmcyErrorCode, EmcyMessage, EmcyProducer};
pub use events::CanOpenEvent;
pub use heartbeat::{HeartbeatEvent, HeartbeatMonitor, HeartbeatMonitorState, HeartbeatProducer};
pub use lss::{LssEvent, LssIdentity, LssMode, LssSlave};
pub use nmt::{NmtCommand, NmtHandler, NmtState, NmtTransition};
#[cfg(feature = "embassy")]
pub use node::SharedNode;
pub use node::{Node, NodeConfig, ResetType};
#[cfg(feature = "embassy")]
pub use od::OdEventSignal;
pub use od::{
    AccessType, ObjectDictionary, ObjectType, OdChanges, OdEntryMeta, OdError, OdEvent,
    OdEventSource,
};
pub use pdo::engine::{
    sync_cyclic, PdoConfigSource, PdoNumber, TransmissionType, EVENT_DRIVEN,
    EVENT_DRIVEN_MANUFACTURER, SYNC_ACYCLIC, SYNC_CYCLIC_1,
};
pub use time::Clock;
pub use transport::{CanError, CanFrame, MailboxTransport};
