<!-- 
NOTICE: THIS FILE HAS BEEN MODIFIED BY Leo Ghafari UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
-->


# Disclaimer
This is a Rust port of the [opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics) C++ library.
The orignal Apache 2.0 license notice is available [here](LICENSE_APACHE.md).


# Intro
A simple, analytical inverse kinematic library for industrial robots with parallel bases and
spherical wrists. Based on the paper `An Analytical Solution of the Inverse Kinematics Problem
of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist` by
Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur.

# Purpose
This package is meant to provide a simpler alternative to IK-Fast based solutions in situations
where one has an industrial robot with a parallel base and spherical wrist. This configuration
is extremely common in industrial robots.

The kinematics are parameterized by 7 primary values taken directly from the robot's spec sheet
and a set of joint-zero offsets. Given this structure, no other setup is required.

# Build Status
TODO

# Parameters

This library makes use of 7 kinematic parameters (a1, a2, b, c1, c2, c3, and c4) defined in the paper `An Analytical Solution of the Inverse Kinematics Problem
of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist`. See the paper for details.

This paper assumes that the arm is at zero when all joints are sticking straight up in the air as seen in the image below. It also assumes that all rotations are positive about the base axis of the robot.

![OPW Diagram](opw.png)

To use the library, fill out an `opwk::Parameters` data structure with the appropriate values for the 7 kinematic parameters and any joint offsets required to bring the paper's zero position (arm up in Z) to the manufacturers position. Additionally, there are 6 "sign correction" parameters (-1 or 1) that should be specified if your robot's axes do not match the convention in the paper.

For example, the ABB IRB2400 has the following values:
```rust
  let p = Parameters {
      a1: 0.100,
      a2: -0.135,
      b: 0.000,
      c1: 0.615,
      c2: 0.705,
      c3: 0.755,
      c4: 0.085,
      offsets: [0.0, 0.0, -std::f32::consts::PI / 2.0, 0.0, 0.0, 0.0],
      sign_corrections: [1, 1, 1, 1, 1, 1],
  };
``` 

Note that the offset of the third joint is -90 degrees, bringing the joint from the upright position to parallel with the ground at "zero".

# Example


# Notes

The library returns the 8 kinematically unique solutions for a given pose. Note that:
 1. These solutions ARE NOT the ONLY solutions. For each joint that can rotate more than 2 * Pi, there exists redundant solutions. For example, if joint 6 can rotate -2*Pi to 2*Pi then a solution with joint 6 at 4.0 radians also has a solution with joint 6 at -2.28 radians and all other values the same.
 2. This library has no concept of LIMITS! Check your own limits. Be sure to check the redundant solutions to see if they are in limits. Consider calling `opwk::harmonizeTowardZero(qs: &mut opwk::types::JointState)` to help check these.
