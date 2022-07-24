pub(crate) mod internal {

    use crate::types::{JointState, Parameters};
    use nalgebra as na;

    pub fn forward(p: &Parameters, qs: &JointState) -> na::IsometryMatrix3<f32> {
        let sc = &p.sign_corrections;
        let corrections: [i8; 6] = sc.into();
        let q = JointState(
            qs.0 * corrections[0] as f32 - p.offsets[0],
            qs.1 * corrections[1] as f32 - p.offsets[1],
            qs.2 * corrections[2] as f32 - p.offsets[2],
            qs.3 * corrections[3] as f32 - p.offsets[3],
            qs.4 * corrections[4] as f32 - p.offsets[4],
            qs.5 * corrections[5] as f32 - p.offsets[5],
        );

        let psi3 = p.a2.atan2(p.c3);
        let k = (p.a2 * p.a2 + p.c3 * p.c3).sqrt();

        let cx1 = p.c2 * q.1.sin() + k * (q.1 + q.2 + psi3).sin() + p.a1;
        let cy1 = p.b;
        let cz1 = p.c2 * q.1.cos() + k * (q.1 + q.2 + psi3).cos();

        let cx0 = cx1 * q.0.cos() - cy1 * q.0.sin();
        let cy0 = cx1 * q.0.sin() + cy1 * q.0.cos();
        let cz0 = cz1 + p.c1;

        let s1 = q.0.sin();
        let s2 = q.1.sin();
        let s3 = q.2.sin();
        let s4 = q.3.sin();
        let s5 = q.4.sin();
        let s6 = q.5.sin();

        let c1 = q.0.cos();
        let c2 = q.1.cos();
        let c3 = q.2.cos();
        let c4 = q.3.cos();
        let c5 = q.4.cos();
        let c6 = q.5.cos();

        let r_0c = na::matrix![
        c1 * c2 * c3 - c1 * s2 * s3,    -s1,    c1 * c2 * s3 + c1 * s2 * c3;
        s1 * c2 * c3 - s1 * s2 * s3,    c1,     s1 * c2 * s3 + s1 * s2 * c3;
        -s2 * c3 - c2 * s3         ,    0.0,    -s2 * s3 + c2 * c3];

        let r_ce = na::matrix![
            c4 * c5 * c6 - s4 * s6,     -c4 * c5 * s6 - s4 * c6,    c4 * s5;
            s4 * c5 * c6 + c4 * s6,     -s4 * c5 * s6 + c4 * c6,    s4 * s5;
            -s5 * c6,                   s5 * s6,                    c5
        ];

        let r_oe = r_0c * r_ce;
        let u = na::vector![cx0, cy0, cz0] + p.c4 * r_oe * na::Vector3::z();

        na::IsometryMatrix3::from_parts(
            na::Translation::from(u),
            na::Rotation3::from_matrix_unchecked(r_oe),
        )
    }
}
