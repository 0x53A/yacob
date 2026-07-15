pub mod engine;

pub use engine::{
    PdoConfigSource, PdoMapping, PdoNumber, RpdoConfig, RpdoEngine, TpdoConfig, TpdoEngine,
    TransmissionType, PDO_MAX_MAPPINGS, PDO_MAX_PAYLOAD_BITS,
};

/// Helper for generated code: create an empty Vec (avoids generated code importing heapless).
pub fn heapless_vec_new<T, const N: usize>() -> heapless::Vec<T, N> {
    heapless::Vec::new()
}

/// Helper for generated code: push to a Vec.
pub fn heapless_vec_push<T, const N: usize>(v: &mut heapless::Vec<T, N>, val: T) -> Result<(), T> {
    v.push(val)
}
