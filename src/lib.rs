mod forward;
mod inverse;
pub mod types;

pub use forward::internal::forward;
pub use inverse::internal::inverse;

pub fn is_valid(qs: &crate::types::JointState) -> bool {
    [qs.0, qs.1, qs.2, qs.3, qs.4, qs.5]
        .iter()
        .all(|q| q.is_finite())
}

pub fn harmonize_toward_zero(qs: &mut crate::types::JointState) {
    for q in [
        &mut qs.0, &mut qs.1, &mut qs.2, &mut qs.3, &mut qs.4, &mut qs.5,
    ] {
        if *q > std::f32::consts::PI {
            *q -= std::f32::consts::PI * 2.0;
        } else if *q < std::f32::consts::PI {
            *q += std::f32::consts::PI * 2.0;
        }
    }
}
