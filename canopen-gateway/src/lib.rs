//! Code-first building blocks for CANopen gateways.
//!
//! This crate deliberately does not know about ROS, MQTT, or any other
//! middleware. A middleware adapter should only have to:
//!
//! - pass received CAN frames through [`PdoToTopic`], then publish the returned
//!   message immediately;
//! - pass subscription messages through [`TopicToPdo`] into a shared
//!   [`LatestPdoSet`], then transmit prepared frames;
//! - pass service requests through [`ServiceToSdo`] and serialize them with a
//!   [`ServiceQueue`] for each SDO server channel.
//!
//! The policies that caused subtle differences between hand-written gateways
//! are explicit here: no implicit inbound buffering, latest-only RPDO slots,
//! mandatory outbound validity, fair selection between PDOs, commit only after
//! transport acceptance, bounded SDO waiting, and generation-checked
//! completion.

mod inbound;
mod outbound;
mod service;

pub use inbound::{FreshState, FreshValue, PdoToTopic, PdoToTopicError};
pub use outbound::{
    ConfirmResult, LatestPdoSet, OfferResult, PdoSlot, PdoSlotError, PreparedPdo, SlotStats,
    TopicToPdo, Validity,
};
pub use service::{
    ActiveService, EnqueueError, QueueDiscipline, SdoCall, SdoFailure, SdoReply, SdoTransfer,
    ServicePoll, ServiceQueue, ServiceQueueConfig, ServiceToSdo, SDO_MAX_DATA,
};
