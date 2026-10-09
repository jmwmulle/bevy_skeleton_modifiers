use bevy_skeleton_modifiers::*;
use glam::{Affine3A, Quat, Vec3};
#[test]
fn axis_helpers_match_godot_table() {
    let axes = [
        BoneAxis::PlusX,
        BoneAxis::MinusX,
        BoneAxis::PlusY,
        BoneAxis::MinusY,
        BoneAxis::PlusZ,
        BoneAxis::MinusZ,
    ];
    for (a, v) in axes.into_iter().zip([
        Vec3::X,
        Vec3::NEG_X,
        Vec3::Y,
        Vec3::NEG_Y,
        Vec3::Z,
        Vec3::NEG_Z,
    ]) {
        assert_eq!(a.vector(), v)
    }
    assert_eq!(BoneDirection::FromParent.vector(Vec3::Y * 2.), Vec3::Y);
    assert_eq!(SecondaryDirection::None.vector(Vec3::X), Vec3::ZERO);
    assert_eq!(RotationAxis::All.vector(Vec3::X), Vec3::ZERO);
}
#[test]
fn influence_zero_is_identity_and_one_is_output() {
    let input = [BonePose::default()];
    let output = [BonePose {
        translation: Vec3::X,
        rotation: Quat::from_rotation_z(1.),
        scale: Vec3::splat(2.),
    }];
    let mut o = output;
    apply_influence(&input, &mut o, 0.);
    assert_eq!(o, input);
    o = output;
    apply_influence(&input, &mut o, 1.);
    assert_eq!(o, output);
}
#[test]
fn chain_walk_rejects_non_descendant_end() {
    let rest = [BonePose::default(); 3];
    let mut local = rest;
    let sk = SkeletonView {
        parents: &[None, Some(0), None],
        rest: &rest,
        local: &mut local,
        root_global: Affine3A::IDENTITY,
    };
    assert_eq!(sk.chain(0, 2), Err(ChainError::NotDescendant));
    assert_eq!(sk.chain(0, 1).unwrap().as_slice(), &[0, 1]);
}
#[test]
fn global_pose_invariant_to_uniform_root_scale() {
    let rest = [
        BonePose::default(),
        BonePose {
            translation: Vec3::Y,
            ..Default::default()
        },
    ];
    for scale in [0.01, 1., 2.] {
        let mut local = rest;
        let world_rot = Quat::from_rotation_z(0.7);
        let mut sk = SkeletonView {
            parents: &[None, Some(0)],
            rest: &rest,
            local: &mut local,
            root_global: Affine3A::from_scale_rotation_translation(
                Vec3::splat(scale),
                world_rot,
                Vec3::X,
            ),
        };
        assert!(
            Vec3::from(sk.global_pose(1).translation)
                .distance(Vec3::X + world_rot * Vec3::Y * scale)
                < 1e-6
        );
        sk.set_global_rotation(0, Quat::IDENTITY);
        assert!(sk.local[0].rotation.dot(world_rot.inverse()).abs() > 1. - 1e-6);
    }
}
#[test]
fn invalid_hierarchies_and_long_chains_are_rejected() {
    let rest = [BonePose::default(); 33];
    let mut local = rest;
    let parents: Vec<_> = (0..33)
        .map(|i| if i == 0 { None } else { Some(i - 1) })
        .collect();
    let sk = SkeletonView {
        parents: &parents,
        rest: &rest,
        local: &mut local,
        root_global: Affine3A::IDENTITY,
    };
    assert_eq!(sk.chain(0, 32), Err(ChainError::TooLong));
    let sk = SkeletonView {
        parents: &[Some(0)],
        rest: &rest[..1],
        local: &mut local[..1],
        root_global: Affine3A::IDENTITY,
    };
    assert_eq!(sk.validate(), Err(ChainError::Cycle));
}
