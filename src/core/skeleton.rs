// Ported from godotengine/godot scene/3d/skeleton_3d.cpp and skeleton_modifier_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Pose and hierarchy storage independent of an engine.
use glam::{Affine3A, Quat, Vec3};
/// A bone's local translation, rotation, and scale.
#[derive(Clone, Copy, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
pub struct BonePose {
    /// Local position in parent coordinates.
    pub translation: Vec3,
    /// Local orientation, stored as an xyzw quaternion.
    pub rotation: Quat,
    /// Local scale; nonuniform scale is unsupported for solving.
    pub scale: Vec3,
}
impl Default for BonePose {
    fn default() -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        }
    }
}
impl BonePose {
    /// Convert to an affine transform.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn affine(self) -> Affine3A {
        super::godot_math::pose_affine(self.scale, self.rotation, self.translation)
    }
}
/// Borrowed skeleton. Bone indices address all three equal-length slices.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
pub struct SkeletonView<'a> {
    /// Parent bone indices; root bones have no parent.
    pub parents: &'a [Option<u16>],
    /// Reference local poses, sharing indices with the other slices.
    pub rest: &'a [BonePose],
    /// Input poses overwritten by the solved local poses.
    pub local: &'a mut [BonePose],
    /// World transform of the skeleton coordinate system.
    pub root_global: Affine3A,
}
/// A validated contiguous ancestry path, with no update-time allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
pub struct ChainIndices {
    len: u8,
    bones: [u16; 32],
}
impl ChainIndices {
    /// The chain in root-to-end order.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn as_slice(&self) -> &[u16] {
        &self.bones[..self.len as usize]
    }
    /// Number of joints.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn len(&self) -> usize {
        self.len as usize
    }
    /// Whether there are no joints.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}
/// Why an ancestry path could not be constructed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
pub enum ChainError {
    /// The chain exceeds 32 joints.
    TooLong,
    /// The end cannot be reached from the specified root.
    NotDescendant,
    /// An index, entity, or slice length is invalid.
    InvalidIndex,
    /// The parent links form a cycle.
    Cycle,
}
impl SkeletonView<'_> {
    /// Validate indices, slice lengths, and acyclic parents before solving.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn validate(&self) -> Result<(), ChainError> {
        let n = self.local.len();
        if self.parents.len() != n || self.rest.len() != n || n > u16::MAX as usize {
            return Err(ChainError::InvalidIndex);
        }
        for i in 0..n {
            let mut p = Some(i as u16);
            for step in 0..=n {
                if let Some(j) = p {
                    if j as usize >= n {
                        return Err(ChainError::InvalidIndex);
                    }
                    if step == n {
                        return Err(ChainError::Cycle);
                    }
                    p = self.parents[j as usize];
                } else {
                    break;
                }
            }
        }
        Ok(())
    }
    /// Build a contiguous path including both ends.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn chain(&self, root: u16, end: u16) -> Result<ChainIndices, ChainError> {
        if root as usize >= self.local.len() || end as usize >= self.local.len() {
            return Err(ChainError::InvalidIndex);
        }
        let mut chain = ChainIndices {
            len: 0,
            bones: [0; 32],
        };
        let mut i = end;
        loop {
            if chain.as_slice().contains(&i) {
                return Err(ChainError::Cycle);
            }
            if chain.len() == 32 {
                return Err(ChainError::TooLong);
            }
            chain.bones[chain.len()] = i;
            chain.len += 1;
            if i == root {
                break;
            }
            i = self
                .parents
                .get(i as usize)
                .copied()
                .flatten()
                .ok_or(ChainError::NotDescendant)?;
            if i as usize >= self.local.len() {
                return Err(ChainError::InvalidIndex);
            }
        }
        let len = chain.len();
        chain.bones[..len].reverse();
        Ok(chain)
    }
    pub(crate) fn pose_from(&self, i: usize, poses: &[BonePose]) -> Affine3A {
        let mut result = poses.get(i).copied().unwrap_or_default().affine();
        let mut p = self.parents.get(i).copied().flatten();
        for _ in 0..poses.len() {
            let Some(j) = p else { return result };
            let Some(pose) = poses.get(j as usize) else {
                return result;
            };
            result = super::godot_math::affine_mul(pose.affine(), result);
            p = self.parents.get(j as usize).copied().flatten();
        }
        result
    }
    /// Pose in skeleton coordinates, before the world/root transform.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn skeleton_pose(&self, i: u16) -> Affine3A {
        self.pose_from(i as usize, self.local)
    }
    /// Current world pose, calculated from locals without propagation latency.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn global_pose(&self, i: u16) -> Affine3A {
        self.root_global * self.skeleton_pose(i)
    }
    /// Set a world rotation, preserving the bone's local translation and scale.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn set_global_rotation(&mut self, i: u16, rotation: Quat) {
        let parent = self
            .parents
            .get(i as usize)
            .copied()
            .flatten()
            .map_or(self.root_global, |p| self.global_pose(p));
        if let Some(local) = self.local.get_mut(i as usize) {
            local.rotation = (super::godot_math::rotation(parent).inverse() * rotation).normalize();
        }
    }
    pub(crate) fn set_skeleton_rotation(&mut self, i: u16, q: Quat) {
        let parent = self.parents[i as usize].map_or(Quat::IDENTITY, |p| {
            super::godot_math::rotation(self.skeleton_pose(p))
        });
        self.local[i as usize].rotation = (parent.inverse() * q).normalize();
    }
}
/// Blend a solved pose with the animation/input pose, as Godot's modifier stack does.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
pub fn apply_influence(input: &[BonePose], output: &mut [BonePose], influence: f32) {
    let t = if influence.is_finite() {
        influence.clamp(0., 1.)
    } else {
        0.
    };
    for (a, b) in input.iter().zip(output.iter_mut()) {
        if t == 0. {
            *b = *a
        } else if t < 1. {
            b.translation = a.translation.lerp(b.translation, t);
            b.rotation = a.rotation.slerp(b.rotation, t).normalize();
            b.scale = a.scale.lerp(b.scale, t);
        }
    }
}
