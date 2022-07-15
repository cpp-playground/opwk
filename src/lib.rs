mod forward;
mod inverse;
pub mod types;

pub use forward::internal::forward;
pub use inverse::internal::inverse;

pub fn is_valid(qs: &crate::types::JointState) -> bool {
    qs.iter().all(|q| q.is_finite())
}

pub fn harmonize_toward_zero(qs: &mut crate::types::JointState) {
    for q in qs {
        if *q > std::f32::consts::PI {
            *q -= std::f32::consts::PI * 2.0;
        } else if *q < std::f32::consts::PI {
            *q += std::f32::consts::PI * 2.0;
        }
    }
}
