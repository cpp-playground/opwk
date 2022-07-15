pub(crate) mod internal {

    use std::ops::AddAssign;

    use crate::types::{JointState, Parameters};
    use nalgebra as na;

    pub fn inverse(params: &Parameters, pose: &na::IsometryMatrix3<f32>) -> [JointState; 8] {
        let matrix = pose.rotation.matrix();
        let c = pose.translation.vector - params.c4 * matrix * na::Vector3::z();

        let nx1 = (c.x * c.x + c.y * c.y - params.b * params.b).sqrt() - params.a1;

        // Compute theta1_i, theta1_ii
        let tmp1 = c.y.atan2(c.x);
        let tmp2 = params.b.atan2(nx1 + params.a1);
        let theta1_i = tmp1 - tmp2;
        let theta1_ii = tmp1 + tmp2 - std::f32::consts::PI;

        // theta2 i through iv
        let tmp3 = c.z - params.c1;
        let s1_2 = nx1 * nx1 + tmp3 * tmp3;

        let tmp4 = nx1 + 2.0 * params.a1;
        let s2_2 = tmp4 * tmp4 + tmp3 * tmp3;
        let kappa_2 = params.a2 * params.a2 + params.c3 * params.c3;

        let c2_2 = params.c2 * params.c2;

        let tmp5 = s1_2 + c2_2 - kappa_2;

        let s1 = s1_2.sqrt();
        let s2 = s2_2.sqrt();

        let tmp13 = (tmp5 / (2.0 * s1 * params.c2)).acos();
        let tmp14 = nx1.atan2(c.z - params.c1);
        let theta2_i = -tmp13 + tmp14;
        let theta2_ii = tmp13 + tmp14;

        let tmp6 = s2_2 + c2_2 - kappa_2;

        let tmp15 = (tmp6 / (2.0 * s2 * params.c2)).acos();
        let tmp16 = (nx1 + 2.0 * params.a1).atan2(c.z - params.c1);
        let theta2_iii = -tmp15 - tmp16;
        let theta2_iv = tmp15 - tmp16;

        // theta3
        let tmp7 = s1_2 - c2_2 - kappa_2;
        let tmp8 = s2_2 - c2_2 - kappa_2;
        let tmp9 = 2.0 * params.c2 * kappa_2.sqrt();
        let tmp10 = params.a2.atan2(params.c3);
        let tmp11 = (tmp7 / tmp9).acos();

        let theta3_i = tmp11 - tmp10;
        let theta3_ii = -tmp11 - tmp10;

        let tmp12 = (tmp8 / tmp9).acos();
        let theta3_iii = tmp12 - tmp10;
        let theta3_iv = -tmp12 - tmp10;

        let sin1 = [
            theta1_i.sin(),
            theta1_i.sin(),
            theta1_ii.sin(), // ???
            theta1_ii.sin(),
        ];

        let cos1 = [
            theta1_i.cos(),
            theta1_i.cos(),
            theta1_ii.cos(), // ???
            theta1_ii.cos(),
        ];

        let s23 = [
            (theta2_i + theta3_i).sin(),
            (theta2_ii + theta3_ii).sin(),
            (theta2_iii + theta3_iii).sin(),
            (theta2_iv + theta3_iv).sin(),
        ];

        let c23 = [
            (theta2_i + theta3_i).cos(),
            (theta2_ii + theta3_ii).cos(),
            (theta2_iii + theta3_iii).cos(),
            (theta2_iv + theta3_iv).cos(),
        ];

        let m = [
            matrix[(0, 2)] * s23[0] * cos1[0]
                + matrix[(1, 2)] * s23[0] * sin1[0]
                + matrix[(2, 2)] * c23[0],
            matrix[(0, 2)] * s23[1] * cos1[1]
                + matrix[(1, 2)] * s23[1] * sin1[1]
                + matrix[(2, 2)] * c23[1],
            matrix[(0, 2)] * s23[2] * cos1[2]
                + matrix[(1, 2)] * s23[2] * sin1[2]
                + matrix[(2, 2)] * c23[2],
            matrix[(0, 2)] * s23[3] * cos1[3]
                + matrix[(1, 2)] * s23[3] * sin1[3]
                + matrix[(2, 2)] * c23[3],
        ];

        let theta5_i = (1.0 - m[0] * m[0]).sqrt().atan2(m[0]);
        let theta5_ii = (1.0 - m[1] * m[1]).sqrt().atan2(m[1]);
        let theta5_iii = (1.0 - m[2] * m[2]).sqrt().atan2(m[2]);
        let theta5_iv = (1.0 - m[3] * m[3]).sqrt().atan2(m[3]);

        let theta5_v = -theta5_i;
        let theta5_vi = -theta5_ii;
        let theta5_vii = -theta5_iii;
        let theta5_viii = -theta5_iv;

        // The derived equations in the paper are geometric and break down when joint 5 is equal to zero.
        // When joint 5 is equal to zeros the values passed to std::atan2 are both zero which is undefined.
        // This can result in significant rotation error up to PI between joint 4 and joint 6 each.
        // In the paper it defines an equation for Rc which is the rotation matrix of joint 5 relative to the base.
        // If you set joint 4 to zero and joint 5 is zero it results in the matrix below where the z-axis is the
        // same as the provided pose. Next, it takes the poses x-axis and projects it onto Rc and then leverage
        // std::atan2 to calculate the joint 6 angle.

        let zero_threshold = 1e-6f32;

        let theta4_i: f32;
        let theta6_i: f32;
        if f32::abs(theta5_i) < zero_threshold {
            (theta4_i, theta6_i) = handle_null_joint_5_case(matrix, theta1_i);
        } else {
            (theta4_i, theta6_i) = handle_nominal_joint_5_case(matrix, 0, &cos1, &sin1, &c23, &s23);
        }

        let theta4_ii: f32;
        let theta6_ii: f32;
        if theta5_ii.abs() < zero_threshold {
            (theta4_ii, theta6_ii) = handle_null_joint_5_case(matrix, theta1_i);
        } else {
            (theta4_ii, theta6_ii) =
                handle_nominal_joint_5_case(matrix, 1, &cos1, &sin1, &c23, &s23);
        }

        let theta4_iii: f32;
        let theta6_iii: f32;
        if theta5_iii.abs() < zero_threshold {
            (theta4_iii, theta6_iii) = handle_null_joint_5_case(matrix, theta1_ii);
        } else {
            (theta4_iii, theta6_iii) =
                handle_nominal_joint_5_case(matrix, 2, &cos1, &sin1, &c23, &s23);
        }

        let theta4_iv: f32;
        let theta6_iv: f32;
        if theta5_iv.abs() < zero_threshold {
            (theta4_iv, theta6_iv) = handle_null_joint_5_case(matrix, theta1_ii);
        } else {
            (theta4_iv, theta6_iv) =
                handle_nominal_joint_5_case(matrix, 3, &cos1, &sin1, &c23, &s23);
        }

        let theta4_v = theta4_i + std::f32::consts::PI;
        let theta4_vi = theta4_ii + std::f32::consts::PI;
        let theta4_vii = theta4_iii + std::f32::consts::PI;
        let theta4_viii = theta4_iv + std::f32::consts::PI;

        let theta6_v = theta6_i - std::f32::consts::PI;
        let theta6_vi = theta6_ii - std::f32::consts::PI;
        let theta6_vii = theta6_iii - std::f32::consts::PI;
        let theta6_viii = theta6_iv - std::f32::consts::PI;

        let theta = na::SMatrix::<f32, 8, 6>::from_iterator(
            [
                theta1_i,
                theta1_i,
                theta1_ii,
                theta1_ii,
                theta1_i,
                theta1_i,
                theta1_ii,
                theta1_ii,
                theta2_i,
                theta2_ii,
                theta2_iii,
                theta2_iv,
                theta2_i,
                theta2_ii,
                theta2_iii,
                theta2_iv,
                theta3_i,
                theta3_ii,
                theta3_iii,
                theta3_iv,
                theta3_i,
                theta3_ii,
                theta3_iii,
                theta3_iv,
                theta4_i,
                theta4_ii,
                theta4_iii,
                theta4_iv,
                theta4_v,
                theta4_vi,
                theta4_vii,
                theta4_viii,
                theta5_i,
                theta5_ii,
                theta5_iii,
                theta5_iv,
                theta5_v,
                theta5_vi,
                theta5_vii,
                theta5_viii,
                theta6_i,
                theta6_ii,
                theta6_iii,
                theta6_iv,
                theta6_v,
                theta6_vi,
                theta6_vii,
                theta6_viii,
            ]
            .iter()
            .cloned(),
        );

        let offsets = na::vector![
            params.offsets[0],
            params.offsets[1],
            params.offsets[2],
            params.offsets[3],
            params.offsets[4],
            params.offsets[5]
        ];

        let signs = na::vector![
            params.sign_corrections[0] as f32,
            params.sign_corrections[1] as f32,
            params.sign_corrections[2] as f32,
            params.sign_corrections[3] as f32,
            params.sign_corrections[4] as f32,
            params.sign_corrections[5] as f32
        ];

        let mut sols = na::SMatrix::<f32, 6, 8>::zeros();
        for (orig_col, mut sol_col) in theta
            .transpose()
            .column_iter_mut()
            .zip(sols.column_iter_mut())
        {
            orig_col.add_to(&offsets, &mut sol_col);
            sol_col.component_mul_assign(&signs);
        }

        sols.data.0
    }

    fn handle_nominal_joint_5_case(
        matrix: &na::Matrix3<f32>,
        index: usize,
        cos1: &[f32; 4],
        sin1: &[f32; 4],
        c23: &[f32; 4],
        s23: &[f32; 4],
    ) -> (f32, f32) {
        let theta4_iiiy = matrix[(1, 2)] * cos1[index] - matrix[(0, 2)] * sin1[index];
        let theta4_iiix = matrix[(0, 2)] * c23[index] * cos1[index]
            + matrix[(1, 2)] * c23[index] * sin1[index]
            - matrix[(2, 2)] * s23[index];

        let theta6_iiiy = matrix[(0, 1)] * s23[index] * cos1[index]
            + matrix[(1, 1)] * s23[index] * sin1[index]
            + matrix[(2, 1)] * c23[index];
        let theta6_iiix = -matrix[(0, 0)] * s23[index] * cos1[index]
            - matrix[(1, 0)] * s23[index] * sin1[index]
            - matrix[(2, 0)] * c23[index];

        (
            theta4_iiiy.atan2(theta4_iiix),
            theta6_iiiy.atan2(theta6_iiix),
        )
    }

    fn handle_null_joint_5_case(matrix: &na::Matrix3<f32>, base_theta: f32) -> (f32, f32) {
        let xe: na::Vector3<f32> = na::vector![matrix[(0, 0)], matrix[(1, 0)], matrix[(2, 0)]];
        let mut rc_mat = na::Matrix3::zeros();
        rc_mat
            .column_mut(1)
            .add_assign(na::vector![-base_theta.sin(), base_theta.cos(), 0.0]);
        // yc
        rc_mat.column_mut(2).add_assign(matrix.column(2));
        // zc and ze are equal
        let tmp = rc_mat.column(1).cross(&rc_mat.column(2));
        rc_mat.column_mut(0).add_assign(tmp);
        // xc
        let xec = rc_mat.transpose() * xe;

        (0.0, xec.y.atan2(xec.x))
    }
}
