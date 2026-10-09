// Ported from godotengine/godot scene/3d/ik_modifier_3d.cpp and spring_bone_simulator_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Godot's named bone and rotation axes.
use glam::Vec3;
/// Signed coordinate axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub enum BoneAxis {
    /// Positive X axis.
    PlusX,
    /// Negative X axis.
    MinusX,
    /// Positive Y axis.
    PlusY,
    /// Negative Y axis.
    MinusY,
    /// Positive Z axis.
    PlusZ,
    /// Negative Z axis.
    MinusZ,
}
impl BoneAxis {
    /// Unit vector for this axis.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn vector(self) -> Vec3 {
        match self {
            Self::PlusX => Vec3::X,
            Self::MinusX => Vec3::NEG_X,
            Self::PlusY => Vec3::Y,
            Self::MinusY => Vec3::NEG_Y,
            Self::PlusZ => Vec3::Z,
            Self::MinusZ => Vec3::NEG_Z,
        }
    }
}
/// Direction for an extended end bone.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub enum BoneDirection {
    /// Positive X axis.
    PlusX,
    /// Negative X axis.
    MinusX,
    /// Positive Y axis.
    PlusY,
    /// Negative Y axis.
    MinusY,
    /// Positive Z axis.
    PlusZ,
    /// Negative Z axis.
    MinusZ,
    #[default]
    /// Infer the forward axis from the parent-to-bone displacement.
    FromParent,
}
impl BoneDirection {
    /// Use the given parent-to-bone vector for `FromParent`.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn vector(self, parent: Vec3) -> Vec3 {
        match self {
            Self::PlusX => Vec3::X,
            Self::MinusX => Vec3::NEG_X,
            Self::PlusY => Vec3::Y,
            Self::MinusY => Vec3::NEG_Y,
            Self::PlusZ => Vec3::Z,
            Self::MinusZ => Vec3::NEG_Z,
            Self::FromParent => parent.normalize_or_zero(),
        }
    }
}
/// Optional secondary (pole or limit-right) direction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub enum SecondaryDirection {
    #[default]
    /// No secondary axis.
    None,
    /// Positive X axis.
    PlusX,
    /// Negative X axis.
    MinusX,
    /// Positive Y axis.
    PlusY,
    /// Negative Y axis.
    MinusY,
    /// Positive Z axis.
    PlusZ,
    /// Negative Z axis.
    MinusZ,
    /// Use the supplied custom vector.
    Custom,
}
impl SecondaryDirection {
    /// Resolve the named or custom vector.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn vector(self, custom: Vec3) -> Vec3 {
        match self {
            Self::None => Vec3::ZERO,
            Self::PlusX => Vec3::X,
            Self::MinusX => Vec3::NEG_X,
            Self::PlusY => Vec3::Y,
            Self::MinusY => Vec3::NEG_Y,
            Self::PlusZ => Vec3::Z,
            Self::MinusZ => Vec3::NEG_Z,
            Self::Custom => custom,
        }
    }
}
/// Restrict rotation to a hinge, or permit all axes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub enum RotationAxis {
    /// Rotate around local X.
    X,
    /// Rotate around local Y.
    Y,
    /// Rotate around local Z.
    Z,
    #[default]
    /// Allow all rotation axes.
    All,
    /// Use the supplied custom vector.
    Custom,
}
impl RotationAxis {
    /// Hinge axis; all axes returns zero.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn vector(self, custom: Vec3) -> Vec3 {
        match self {
            Self::X => Vec3::X,
            Self::Y => Vec3::Y,
            Self::Z => Vec3::Z,
            Self::All => Vec3::ZERO,
            Self::Custom => custom,
        }
    }
}

// Godot Basis::xform_inv is the transpose, rather than a general matrix inverse.
pub(super) fn end_direction(
    sk: &super::SkeletonView<'_>,
    bone: u16,
    direction: BoneDirection,
) -> Vec3 {
    let i = bone as usize;
    let parent = sk.rest[i].affine().matrix3.transpose() * sk.local[i].translation;
    direction.vector(parent)
}
