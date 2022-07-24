#[cfg(test)]
mod tests {

    use opwk::types::*;
    use opwk::{forward, inverse, is_valid};

    use nalgebra as na;

    fn kuka_kr6_params() -> Parameters {
        Parameters {
            a1: 0.025,
            a2: -0.035,
            b: 0.000,
            c1: 0.400,
            c2: 0.315,
            c3: 0.365,
            c4: 0.080,
            offsets: [0.0, -std::f32::consts::PI / 2.0, 0.0, 0.0, 0.0, 0.0],
            sign_corrections: SignCorrections(
                RotationDirection::Negative,
                RotationDirection::Positive,
                RotationDirection::Positive,
                RotationDirection::Negative,
                RotationDirection::Positive,
                RotationDirection::Negative,
            ),
        }
    }

    fn compare_poses(a: &na::IsometryMatrix3<f32>, b: &na::IsometryMatrix3<f32>) {
        let ra = a.rotation.matrix();
        let rb = b.rotation.matrix();

        let tolerance = 1e-5f32;

        for i in 0..3 {
            for j in 0..3 {
                expect_near(ra[(i, j)], rb[(i, j)], tolerance);
            }
        }

        expect_near(a.translation.x, b.translation.x, tolerance);
        expect_near(a.translation.y, b.translation.y, tolerance);
        expect_near(a.translation.z, b.translation.z, tolerance);
    }

    fn expect_near(a: f32, b: f32, tolerance: f32) {
        assert!((a - b).abs() < tolerance);
    }

    #[test]
    fn kuka_kr6_forward() {
        let params = kuka_kr6_params();

        let joint_values = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
        let forward_pose = forward(&params, &joint_values.into());

        let expected = na::IsometryMatrix3::<f32>::from_parts(
            na::Translation::from(na::vector![0.7341169, -0.1520347, 0.182639]),
            na::Rotation3::from_matrix_unchecked(na::matrix![
                -0.5965795, 0.000371195, 0.8025539;
                -0.2724458, 0.9405218, -0.202958;
                -0.7548948, -0.3397331, -0.5609949]),
        );

        compare_poses(&forward_pose, &expected);
    }

    #[test]
    fn kuka_kr6_inverse() {
        let params = kuka_kr6_params();

        let joint_values = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
        let forward_pose = forward(&params, &joint_values.into());

        let sols = inverse(&params, &forward_pose);
        for s in sols {
            if is_valid(&s) {
                // Forward kinematics of a solution should result in the same pose
                let pose = forward(&params, &s);
                compare_poses(&forward_pose, &pose);
            }
        }
    }
}
