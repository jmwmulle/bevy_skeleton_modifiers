// Ported from godotengine/godot scene/3d/two_bone_ik_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Analytic two-bone IK derived from Godot's TwoBoneIK3D.
use super::{BoneDirection, BonePose, SecondaryDirection, SkeletonView, godot_math::*};
use glam::{Affine3A, Quat, Vec3};
/// End extension and optional bone-local pole alignment.
#[derive(Clone, Copy, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_twoboneik3d.html).
pub struct TwoBoneSettings {
    /// Optional bone-local axis aligned with the pole plane.
    pub pole_direction: SecondaryDirection,
    /// Custom pole axis in bone coordinates.
    pub pole_direction_vector: Vec3,
    /// Use an extension of the middle joint as the effector.
    pub use_virtual_end: bool,
    /// Simulate a segment beyond the terminal joint.
    pub extend_end_bone: bool,
    /// Direction of the terminal extension.
    pub end_bone_direction: BoneDirection,
    /// Length of the terminal extension in skeleton units.
    pub end_bone_length: f32,
}
impl Default for TwoBoneSettings {
    fn default() -> Self {
        Self {
            pole_direction: SecondaryDirection::None,
            pole_direction_vector: Vec3::ZERO,
            use_virtual_end: false,
            extend_end_bone: false,
            end_bone_direction: BoneDirection::FromParent,
            end_bone_length: 0.,
        }
    }
}
fn mutable_rest(sk: &SkeletonView<'_>, bone: u16, root: u16) -> Affine3A {
    let mut i = bone;
    let mut result = Affine3A::IDENTITY;
    for _ in 0..sk.local.len() {
        let r = sk.rest[i as usize];
        let local = BonePose {
            translation: sk.local[i as usize].translation,
            ..r
        };
        result = local.affine() * result;
        if i == root {
            let parent = sk.parents[i as usize]
                .map_or(Affine3A::IDENTITY, |p| sk.pose_from(p as usize, sk.rest));
            return parent * result;
        }
        let Some(parent) = sk.parents[i as usize] else {
            break;
        };
        i = parent;
    }
    result
}
/// Solve with target and optional pole in skeleton coordinates.
/// Missing or collinear poles use the input bend direction, then a deterministic perpendicular.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_twoboneik3d.html).
pub fn process_two_bone(
    sk: &mut SkeletonView<'_>,
    root: u16,
    middle: u16,
    end: u16,
    target: Vec3,
    pole: Option<Vec3>,
    settings: &TwoBoneSettings,
) {
    if sk.validate().is_err()
        || sk.chain(root, middle).is_err()
        || (!settings.use_virtual_end && sk.chain(middle, end).is_err())
        || !target.is_finite()
    {
        return;
    }
    let valid_end = if settings.use_virtual_end {
        middle
    } else {
        end
    };
    let end_axis = super::axes::end_direction(sk, valid_end, settings.end_bone_direction);
    let extended = settings.extend_end_bone && settings.end_bone_length > 0.;
    if settings.use_virtual_end && !extended {
        return;
    }
    let extension = if extended {
        end_axis * settings.end_bone_length
    } else {
        Vec3::ZERO
    };
    let root_rest = mutable_rest(sk, root, root);
    let mid_rest = mutable_rest(sk, middle, root);
    let end_rest = mutable_rest(sk, valid_end, root).transform_point3(extension);
    let root_axis = Vec3::from(mid_rest.translation) - Vec3::from(root_rest.translation);
    let mid_axis = end_rest - Vec3::from(mid_rest.translation);
    let l1 = root_axis.length();
    let l2 = mid_axis.length();
    if l1 < 1e-8 || l2 < 1e-8 {
        return;
    }
    let root_forward =
        (rotation(sk.pose_from(root as usize, sk.rest)).inverse() * root_axis).normalize_or_zero();
    let mid_forward =
        (rotation(sk.pose_from(middle as usize, sk.rest)).inverse() * mid_axis).normalize_or_zero();
    let root_pose = sk.skeleton_pose(root);
    let mid_pose = sk.skeleton_pose(middle);
    let a = Vec3::from(root_pose.translation);
    let mut destination = target;
    let diff = destination - a;
    let distance = diff.length();
    if distance < 1e-8 {
        return;
    }
    let u = diff / distance;
    let parent =
        sk.parents[root as usize].map_or(Quat::IDENTITY, |p| rotation(sk.skeleton_pose(p)));
    let root_input = rotation(sk.local[root as usize].affine());
    let middle_relative = (rotation(root_pose).inverse() * rotation(mid_pose)).normalize();
    let mut end_pos = destination;
    let middle_pos;
    if distance >= l1 + l2 {
        middle_pos = a + u * l1;
        end_pos = middle_pos + u * l2;
    } else {
        let min_distance = (l1 - l2).abs();
        if distance < min_distance {
            destination = a + u * min_distance;
            end_pos = destination;
        }
        let d = (destination - a).length() as f64;
        let x = (d * d + (l1 as f64).powi(2) - (l2 as f64).powi(2)) / (2. * d);
        let height = ((l1 as f64).powi(2) - x * x).max(0.).sqrt() as f32;
        let provided = pole.unwrap_or(Vec3::from(mid_pose.translation));
        let mut p = projected_normal(a, destination, provided);
        if p == Vec3::ZERO {
            p = projected_normal(a, destination, Vec3::from(mid_pose.translation));
            if p == Vec3::ZERO {
                p = u
                    .cross(if u.x.abs() < 0.9 { Vec3::X } else { Vec3::Y })
                    .normalize_or_zero();
            }
        }
        let center = a + u * (x as f32);
        let plus = center + p * height;
        let minus = center - p * height;
        middle_pos = if plus.distance_squared(provided) < minus.distance_squared(provided) {
            plus
        } else {
            minus
        };
    }
    let root_vector = middle_pos - a;
    let mid_vector = end_pos - middle_pos;
    let root_grest = (parent * root_input).normalize();
    let mut root_local = root_input
        * swing(
            arc(
                root_forward,
                root_grest.inverse() * root_vector.normalize_or_zero(),
            ),
            root_forward,
        );
    let mut root_global = (parent * root_local).normalize();
    let mid_grest = (root_global * middle_relative).normalize();
    let mut mid_local = middle_relative
        * swing(
            arc(
                mid_forward,
                mid_grest.inverse() * mid_vector.normalize_or_zero(),
            ),
            mid_forward,
        );
    let mut mid_global = (root_global * mid_local).normalize();
    let pole_axis = settings
        .pole_direction
        .vector(settings.pole_direction_vector);
    if pole_axis.length_squared() > 1e-10 {
        let provided = pole.unwrap_or(Vec3::from(mid_pose.translation));
        let pole_dir = projected_normal(a, end_pos, provided);
        let axis = mid_vector.normalize_or_zero();
        let k = (mid_global * pole_axis).normalize_or_zero();
        let n = pole_dir
            .cross(root_vector.normalize_or_zero())
            .normalize_or_zero();
        if axis != Vec3::ZERO && k != Vec3::ZERO && n != Vec3::ZERO && n.dot(k).abs() > 1e-5 {
            let c0 = n.dot(k - axis * k.dot(axis)) as f64;
            let c1 = n.dot(axis.cross(k)) as f64;
            let c2 = (n.dot(axis) * k.dot(axis)) as f64;
            let r = (c0 * c0 + c1 * c1).sqrt();
            if r > 1e-10 {
                let phi = c1.atan2(c0);
                let acos = (-c2 / r).clamp(-1., 1.).acos();
                let t1 = (phi + acos) as f32;
                let t2 = (phi - acos) as f32;
                let projected = plane(n, pole_dir).normalize_or_zero();
                let s1 = plane(n, Quat::from_axis_angle(axis, t1) * k)
                    .normalize_or_zero()
                    .dot(projected);
                let s2 = plane(n, Quat::from_axis_angle(axis, t2) * k)
                    .normalize_or_zero()
                    .dot(projected);
                let t = if s1 >= s2 { t1 } else { t2 };
                let root_roll = Quat::from_axis_angle(root_forward, t);
                let mid_roll = Quat::from_axis_angle(mid_forward, t);
                root_local *= root_roll;
                root_global = (parent * root_local).normalize();
                mid_local = root_roll.inverse() * mid_local * mid_roll;
                mid_global = (root_global * mid_local).normalize();
            }
        }
    }
    if root_local.is_finite() && mid_global.is_finite() {
        sk.local[root as usize].rotation = root_local.normalize();
        sk.set_skeleton_rotation(middle, mid_global);
    }
}
