// Ported from godotengine/godot scene/3d/spring_bone_collision_{sphere,capsule,plane}_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Analytic colliders derived from Godot's SpringBoneCollision3D family.
use super::godot_math::rotation;
use glam::{Affine3A, Vec3};
/// Collider geometry. Poses supplied to the spring solver are world transforms.
#[derive(Clone, Copy, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonecollision3d.html).
pub enum SpringCollider {
    /// Sphere, optionally containing the bone tail rather than excluding it.
    Sphere {
        /// Sphere radius in skeleton units.
        radius: f32,
        /// Contain tails inside this sphere.
        inside: bool,
    },
    /// Y-aligned capsule; height includes the two hemispheres.
    Capsule {
        /// Hemisphere radius in skeleton units.
        radius: f32,
        /// Total height including the hemispheres.
        height: f32,
        /// Contain tails inside this shape.
        inside: bool,
    },
    /// Half-space above the pose's local Y plane.
    Plane,
}
impl SpringCollider {
    /// Project a tail out of penetration. Radius is in the same units as the pose.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonecollision3d.html).
    pub fn collide(self, pose: Affine3A, bone_radius: f32, current: Vec3) -> Vec3 {
        let origin = Vec3::from(pose.translation);
        match self {
            Self::Sphere { radius, inside } => sphere(origin, radius, inside, bone_radius, current),
            Self::Capsule {
                radius,
                height,
                inside,
            } => {
                let r = radius.max(0.);
                let offset = pose.transform_vector3(Vec3::Y * (height.max(2. * r) * 0.5 - r));
                let head = origin + offset;
                let tail = origin - offset;
                let v = tail - head;
                let dot = (current - head).dot(v);
                let d = v.length_squared();
                if dot <= 0. {
                    sphere(head, r, inside, bone_radius, current)
                } else if d == 0. {
                    current
                } else if dot >= d {
                    sphere(tail, r, inside, bone_radius, current)
                } else {
                    sphere(head + v * (dot / d), r, inside, bone_radius, current)
                }
            }
            Self::Plane => {
                let n = rotation(pose) * Vec3::Y;
                let distance = (current - origin).dot(n) - bone_radius;
                if distance > 0. {
                    current
                } else {
                    current - n * distance
                }
            }
        }
    }
}
fn sphere(origin: Vec3, radius: f32, inside: bool, bone_radius: f32, current: Vec3) -> Vec3 {
    let diff = current - origin;
    let length = diff.length();
    let r = if inside {
        (radius - bone_radius).max(0.)
    } else {
        (radius + bone_radius).max(0.)
    };
    let distance = if inside { r - length } else { length - r };
    if distance > 0. {
        current
    } else {
        origin + diff.normalize_or_zero() * r
    }
}
