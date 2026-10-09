// Ported from godotengine/godot scene/resources/3d/joint_limitation_cone_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Cone constraints derived from Godot JointLimitationCone3D.
use super::{
    SecondaryDirection,
    godot_math::{arc, matrix_quat, qmul, qnorm, qx, vcross, vdot, vnorm},
};
use glam::{Mat3, Quat, Vec3};
/// A directional constraint in a joint's input/rest coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_jointlimitationcone3d.html).
pub enum JointLimit {
    /// Full cone opening angle, in radians (the maximum swing is half of this).
    Cone {
        /// Full cone opening angle in radians.
        angle: f32,
        /// Optional axis defining the cone frame.
        right_axis: SecondaryDirection,
        /// Custom axis defining the cone frame.
        right_axis_vector: Vec3,
        /// Additional orientation of the cone frame.
        rotation_offset: Quat,
    },
}
impl JointLimit {
    /// Clamp a local direction, preserving its length.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_jointlimitationcone3d.html).
    pub fn solve(self, forward: Vec3, direction: Vec3) -> Vec3 {
        self.solve_direction(forward, direction) * direction.length()
    }
    pub(crate) fn solve_direction(self, forward: Vec3, direction: Vec3) -> Vec3 {
        let Self::Cone {
            angle,
            right_axis,
            right_axis_vector,
            rotation_offset,
        } = self;
        let len = direction.length();
        if len < 1e-5 {
            return vnorm(direction);
        }
        let y = vnorm(forward);
        if y == Vec3::ZERO {
            return vnorm(direction);
        }
        let x = vnorm(right_axis.vector(right_axis_vector));
        let base = if x == Vec3::ZERO || x.dot(y).abs() > 1. - 1e-5 {
            arc(Vec3::Y, y)
        } else {
            let z = vnorm(vcross(x, y));
            let x = vnorm(vcross(y, z));
            matrix_quat(Mat3::from_cols(x, y, z))
        };
        let offset = if rotation_offset.length_squared() > 1e-10 {
            qnorm(rotation_offset)
        } else {
            Quat::IDENTITY
        };
        let space = qnorm(qmul(base, offset));
        let dir = qx(space.inverse(), vnorm(direction));
        let current = vcross(dir, Vec3::Y).length().atan2(vdot(dir, Vec3::Y));
        let maximum = angle.clamp(0., std::f32::consts::TAU) * 0.5;
        if current <= maximum {
            return qx(space, dir);
        }
        let projection = dir - Vec3::Y * dir.y;
        let axis = if projection.length_squared() > 1e-5 {
            vnorm(vcross(Vec3::Y, vnorm(projection)))
        } else if ((current as f64) - std::f64::consts::PI).abs()
            < 1e-5 * (current as f64).abs().max(1.)
        {
            Vec3::NEG_Z
        } else {
            vnorm(vcross(Vec3::Y, dir))
        };
        if axis == Vec3::ZERO {
            return vnorm(direction);
        }
        qx(
            space,
            vnorm(qx(Quat::from_axis_angle(axis, maximum), Vec3::Y)),
        )
    }
}
