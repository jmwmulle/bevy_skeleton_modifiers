// Ported from godotengine/godot scene/3d/spring_bone_simulator_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Verlet spring chains from Godot's SpringBoneSimulator3D.
use super::{
    BoneDirection, ChainIndices, RotationAxis, SkeletonView, SpringCollider, godot_math::*,
};
use glam::{Affine3A, Quat, Vec3};
/// Spatial weighting evaluated along a chain.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
pub enum ChainCurve {
    #[default]
    /// A multiplier of one throughout the chain.
    Constant,
    /// Interpolate between endpoint multipliers.
    Linear {
        /// Multiplier at the root.
        start: f32,
        /// Multiplier at the end.
        end: f32,
    },
    /// Sixteen samples, linearly interpolated.
    Sampled([f32; 16]),
}
impl ChainCurve {
    /// Evaluate at a clamped normalized position.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
    pub fn sample(self, t: f32) -> f32 {
        let t = t.clamp(0., 1.);
        match self {
            Self::Constant => 1.,
            Self::Linear { start, end } => start + (end - start) * t,
            Self::Sampled(values) => {
                let p = t * 15.;
                let i = (p as usize).min(14);
                values[i] + (values[i + 1] - values[i]) * (p - i as f32)
            }
        }
    }
}
/// Reference frame used for persistent tail positions.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
pub enum SpringCenter {
    #[default]
    /// Keep tails in world axes, normalized to skeleton units.
    WorldOrigin,
    /// Keep tails relative to this world transform.
    Node(Affine3A),
    /// Keep tails relative to a skeleton-space bone pose.
    Bone(u16),
}
/// Per-joint overrides used when `individual_config` is enabled.
#[derive(Clone, Copy, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
pub struct SpringJointSettings {
    /// Restoring strength applied to the tail each second.
    pub stiffness: f32,
    /// Fraction of Verlet motion discarded per step.
    pub drag: f32,
    /// Strength of the gravity contribution.
    pub gravity: f32,
    /// Direction of gravity in world axes.
    pub gravity_direction: Vec3,
    /// Bone or collider radius in skeleton units.
    pub radius: f32,
    /// Permitted rotation axis; All allows unrestricted swing.
    pub rotation_axis: RotationAxis,
    /// Custom hinge axis in bone coordinates.
    pub rotation_axis_vector: Vec3,
}
impl Default for SpringJointSettings {
    fn default() -> Self {
        Self {
            stiffness: 1.,
            drag: 0.4,
            gravity: 0.,
            gravity_direction: Vec3::NEG_Y,
            radius: 0.02,
            rotation_axis: RotationAxis::All,
            rotation_axis_vector: Vec3::X,
        }
    }
}
/// Spring settings. Defaults match Godot 4.7.2.
#[derive(Clone, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
pub struct SpringBoneSettings {
    /// Restoring strength applied to the tail each second.
    pub stiffness: f32,
    /// Fraction of Verlet motion discarded per step.
    pub drag: f32,
    /// Strength of the gravity contribution.
    pub gravity: f32,
    /// Direction of gravity in world axes.
    pub gravity_direction: Vec3,
    /// Bone or collider radius in skeleton units.
    pub radius: f32,
    /// Permitted rotation axis; All allows unrestricted swing.
    pub rotation_axis: RotationAxis,
    /// Custom hinge axis in bone coordinates.
    pub rotation_axis_vector: Vec3,
    /// Simulate a segment beyond the terminal joint.
    pub extend_end_bone: bool,
    /// Direction of the terminal extension.
    pub end_bone_direction: BoneDirection,
    /// Length of the terminal extension in skeleton units.
    pub end_bone_length: f32,
    /// Frame in which persistent tail positions are stored.
    pub center_from: SpringCenter,
    /// Use explicit joint overrides when they are present.
    pub individual_config: bool,
    /// Joint settings in root-to-end order, allocated during setup.
    pub joint_overrides: Vec<SpringJointSettings>,
    /// Multiplier on stiffness along the chain.
    pub stiffness_curve: ChainCurve,
    /// Multiplier on drag along the chain.
    pub drag_curve: ChainCurve,
    /// Multiplier on gravity along the chain.
    pub gravity_curve: ChainCurve,
    /// Multiplier on radius along the chain.
    pub radius_curve: ChainCurve,
}
impl Default for SpringBoneSettings {
    fn default() -> Self {
        let j = SpringJointSettings::default();
        Self {
            stiffness: j.stiffness,
            drag: j.drag,
            gravity: j.gravity,
            gravity_direction: j.gravity_direction,
            radius: j.radius,
            rotation_axis: j.rotation_axis,
            rotation_axis_vector: j.rotation_axis_vector,
            extend_end_bone: false,
            end_bone_direction: BoneDirection::FromParent,
            end_bone_length: 0.,
            center_from: SpringCenter::WorldOrigin,
            individual_config: false,
            joint_overrides: Vec::new(),
            stiffness_curve: ChainCurve::Constant,
            drag_curve: ChainCurve::Constant,
            gravity_curve: ChainCurve::Constant,
            radius_curve: ChainCurve::Constant,
        }
    }
}
impl SpringBoneSettings {
    fn joint(&self, i: usize, n: usize) -> SpringJointSettings {
        if self.individual_config
            && let Some(j) = self.joint_overrides.get(i)
        {
            return *j;
        }
        let t = if n > 1 { i as f32 / (n - 1) as f32 } else { 0. };
        SpringJointSettings {
            stiffness: self.stiffness * self.stiffness_curve.sample(t),
            drag: self.drag * self.drag_curve.sample(t),
            gravity: self.gravity * self.gravity_curve.sample(t),
            gravity_direction: self.gravity_direction,
            radius: self.radius * self.radius_curve.sample(t),
            rotation_axis: self.rotation_axis,
            rotation_axis_vector: self.rotation_axis_vector,
        }
    }
}
#[derive(Clone, Copy, Debug)]
struct Verlet {
    tail: Vec3,
    previous: Vec3,
    rotation: Quat,
}
impl Default for Verlet {
    fn default() -> Self {
        Self {
            tail: Vec3::ZERO,
            previous: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        }
    }
}
/// Persistent fixed-capacity simulation storage; call reset after changing the chain.
#[derive(Clone, Debug, Default)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
pub struct SpringBoneState {
    joints: [Verlet; 32],
    initialized: bool,
}
impl SpringBoneState {
    /// Initialize tails from the next input pose.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
    pub fn reset(&mut self) {
        self.initialized = false;
    }
}
/// Advance a chain in place. Nonpositive/nonfinite delta freezes the state and pose.
/// Collider poses are world transforms; gravity and external force are world directions.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html).
pub fn process_spring_bones(
    skel: &mut SkeletonView<'_>,
    chain: &ChainIndices,
    settings: &SpringBoneSettings,
    state: &mut SpringBoneState,
    colliders: &[(SpringCollider, Affine3A)],
    external_force: Vec3,
    dt: f32,
) {
    if !dt.is_finite()
        || dt <= 0.
        || skel.validate().is_err()
        || chain
            .as_slice()
            .iter()
            .any(|&i| i as usize >= skel.local.len())
    {
        return;
    }
    let original_center = match settings.center_from {
        SpringCenter::WorldOrigin => skel.root_global,
        SpringCenter::Node(world) => world.inverse() * skel.root_global,
        SpringCenter::Bone(i) if (i as usize) < skel.local.len() => skel.skeleton_pose(i),
        SpringCenter::Bone(_) => return,
    };
    // Store simulation coordinates in bone units, so a uniform asset scale does
    // not change the length constraint or force response.
    let sx = original_center.matrix3.x_axis.length();
    let sy = original_center.matrix3.y_axis.length();
    let sz = original_center.matrix3.z_axis.length();
    if !sx.is_finite() || sx < 1e-8 || (sx - sy).abs() > 1e-4 * sx || (sx - sz).abs() > 1e-4 * sx {
        return;
    }
    let normalize_space = |a: Affine3A| Affine3A {
        matrix3: a.matrix3 / sx,
        translation: a.translation / sx,
    };
    let center = normalize_space(original_center);
    let inv = center.inverse();
    let center_rot = rotation(center);
    let n = chain.len();
    if !state.initialized {
        for (j, &bone) in chain.as_slice().iter().enumerate() {
            let axis = if j + 1 < n {
                skel.local[chain.as_slice()[j + 1] as usize].translation
            } else if settings.extend_end_bone {
                super::axes::end_direction(skel, bone, settings.end_bone_direction)
                    * settings.end_bone_length
            } else {
                continue;
            };
            let tail = center.transform_point3(skel.skeleton_pose(bone).transform_point3(axis));
            state.joints[j] = Verlet {
                tail,
                previous: tail,
                rotation: Quat::IDENTITY,
            };
        }
    }
    for (j, &bone) in chain.as_slice().iter().enumerate() {
        let axis = if j + 1 < n {
            skel.local[chain.as_slice()[j + 1] as usize].translation
        } else if settings.extend_end_bone && settings.end_bone_length > 0. {
            super::axes::end_direction(skel, bone, settings.end_bone_direction)
                * settings.end_bone_length
        } else {
            continue;
        };
        let len = axis.length();
        if len < 1e-8 {
            continue;
        }
        let config = settings.joint(j, n);
        let hinge = config.rotation_axis.vector(config.rotation_axis_vector);
        let forward = plane(hinge, axis.normalize());
        let global = skel.skeleton_pose(bone);
        let current_rot = rotation(global);
        let world = center * global;
        let origin = Vec3::from(world.translation);
        let v = &mut state.joints[j];
        let external = center_rot.inverse()
            * ((external_force + config.gravity_direction * config.gravity) * dt);
        let mut next = v.tail
            + (v.tail - v.previous) * (1. - config.drag)
            + center_rot * (current_rot * (forward * (config.stiffness * dt)) + external);
        let project = |p: Vec3| {
            if config.rotation_axis == RotationAxis::All {
                p
            } else {
                let q = rotation(world);
                origin + q * plane(hinge, q.inverse() * (p - origin))
            }
        };
        next = limit_length(origin, project(next), len);
        for &(col, pose) in colliders {
            let collider_pose =
                normalize_space(skel.root_global.inverse() * original_center * pose);
            next = col.collide(collider_pose, config.radius, next);
            next = limit_length(origin, project(next), len);
        }
        v.previous = v.tail;
        v.tail = next;
        let from = (current_rot * forward).normalize_or_zero();
        let to = inv.transform_vector3(next - origin).normalize_or_zero();
        let q = from_to(from, to, v.rotation);
        v.rotation = q;
        let desired = q * current_rot;
        if desired.is_finite() {
            skel.set_skeleton_rotation(bone, desired)
        }
    }
    state.initialized = true;
}
