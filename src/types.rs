#[derive(Copy, Clone)]
pub enum RotationDirection {
    Positive = 1,
    Negative = -1,
}

impl From<RotationDirection> for i8 {
    fn from(data: RotationDirection) -> Self {
        data as i8
    }
}

impl Default for RotationDirection {
    fn default() -> Self {
        Self::Positive
    }
}

#[derive(Default)]
pub struct SignCorrections(
    pub RotationDirection,
    pub RotationDirection,
    pub RotationDirection,
    pub RotationDirection,
    pub RotationDirection,
    pub RotationDirection,
);

impl From<&SignCorrections> for [i8; 6] {
    fn from(data: &SignCorrections) -> Self {
        [
            data.0.into(),
            data.1.into(),
            data.2.into(),
            data.3.into(),
            data.4.into(),
            data.5.into(),
        ]
    }
}

pub struct JointState(pub f32, pub f32, pub f32, pub f32, pub f32, pub f32);

impl From<[f32; 6]> for JointState {
    fn from(data: [f32; 6]) -> Self {
        JointState(data[0], data[1], data[2], data[3], data[4], data[5])
    }
}

pub struct Parameters {
    pub a1: f32,
    pub a2: f32,
    pub b: f32,
    pub c1: f32,
    pub c2: f32,
    pub c3: f32,
    pub c4: f32,

    pub offsets: [f32; 6],
    pub sign_corrections: SignCorrections,
}
