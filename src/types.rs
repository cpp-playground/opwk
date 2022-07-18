use std::fmt;

pub type JointState = [f32; 6];
pub struct Parameters {
    pub a1: f32,
    pub a2: f32,
    pub b: f32,
    pub c1: f32,
    pub c2: f32,
    pub c3: f32,
    pub c4: f32,

    pub offsets: [f32; 6],
    pub sign_corrections: [i8; 6],
}

impl fmt::Display for Parameters {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(
            f,
            "Distances: [{} {} {} {} {} {} {}]",
            self.a1, self.a2, self.b, self.c1, self.c2, self.c3, self.c4
        )?;

        write!(f, "Offsets = [")?;
        for offset in self.offsets.iter() {
            write!(f, "{} ", offset)?;
        }
        write!(f, "]\nSign_corrections = [")?;
        for sign_correction in self.sign_corrections.iter() {
            write!(f, "{} ", sign_correction)?;
        }
        write!(f, "]")
    }
}
