use criterion::{black_box, criterion_group, criterion_main, Criterion};

use opwk::forward;
use opwk::inverse;
use opwk::types::*;
use std::f32::consts::PI;

fn bench_forward(c: &mut Criterion) {
    let params: Parameters = Parameters {
        a1: 0.025,
        a2: -0.035,
        b: 0.000,
        c1: 0.400,
        c2: 0.315,
        c3: 0.365,
        c4: 0.080,
        offsets: [0.0, -PI / 2.0, 0.0, 0.0, 0.0, 0.0],
        sign_corrections: SignCorrections(
            RotationDirection::Negative,
            RotationDirection::Positive,
            RotationDirection::Positive,
            RotationDirection::Negative,
            RotationDirection::Positive,
            RotationDirection::Negative,
        ),
    };

    let joint_values: [f32; 6] = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
    c.bench_function("forward kinematics", |b| {
        b.iter(|| forward(black_box(&params), black_box(&joint_values.into())))
    });
}

fn bench_inverse(c: &mut Criterion) {
    let params: Parameters = Parameters {
        a1: 0.025,
        a2: -0.035,
        b: 0.000,
        c1: 0.400,
        c2: 0.315,
        c3: 0.365,
        c4: 0.080,
        offsets: [0.0, -PI / 2.0, 0.0, 0.0, 0.0, 0.0],
        sign_corrections: SignCorrections(
            RotationDirection::Negative,
            RotationDirection::Positive,
            RotationDirection::Positive,
            RotationDirection::Negative,
            RotationDirection::Positive,
            RotationDirection::Negative,
        ),
    };
    let joint_values: [f32; 6] = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
    let pose = forward(&params, &joint_values.into());
    c.bench_function("inverse kinematics", |b| {
        b.iter(|| inverse(black_box(&params), black_box(&pose)))
    });
}

criterion_group!(benches, bench_forward, bench_inverse);
criterion_main!(benches);
