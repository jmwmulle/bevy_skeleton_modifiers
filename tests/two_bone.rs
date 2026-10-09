mod common;
use bevy_skeleton_modifiers::*;
use common::*;
use glam::{Affine3A, Quat, Vec3};
#[test]
fn two_bone_scenarios_match_godot() {
    for name in ["two_sweep", "two_virtual"] {
        let g = golden(name);
        let (parents, rest) = skeleton_data(&g);
        let settings = TwoBoneSettings {
            use_virtual_end: name == "two_virtual",
            extend_end_bone: name == "two_virtual",
            end_bone_direction: BoneDirection::PlusY,
            end_bone_length: 0.5,
            ..Default::default()
        };
        for (i, f) in g["frames"].as_array().unwrap().iter().enumerate() {
            let mut local: Vec<_> = f["input"].as_array().unwrap().iter().map(pose).collect();
            let mut sk = SkeletonView {
                parents: &parents,
                rest: &rest,
                local: &mut local,
                root_global: affine(&f["root"]),
            };
            process_two_bone(
                &mut sk,
                0,
                1,
                2,
                vec(&f["target"]),
                Some(vec(&f["pole"])),
                &settings,
            );
            compare(&sk, f, name, i, 1e-3);
        }
    }
}
#[test]
fn two_bone_degenerate_inputs_are_finite_and_deterministic() {
    for length in [0., 0.5] {
        let rest = [
            BonePose::default(),
            BonePose {
                translation: Vec3::Y * length,
                ..Default::default()
            },
            BonePose {
                translation: Vec3::Y * length,
                ..Default::default()
            },
        ];
        for target in [Vec3::ZERO, Vec3::Y * 0.5, Vec3::Y * 100., Vec3::X * 1e-9] {
            let mut outcomes = Vec::new();
            for _ in 0..2 {
                let mut local = rest;
                let mut sk = SkeletonView {
                    parents: &[None, Some(0), Some(1)],
                    rest: &rest,
                    local: &mut local,
                    root_global: Affine3A::IDENTITY,
                };
                process_two_bone(
                    &mut sk,
                    0,
                    1,
                    2,
                    target,
                    Some(Vec3::Y),
                    &TwoBoneSettings::default(),
                );
                assert!(sk.local.iter().all(|p| p.rotation.is_finite()));
                outcomes.push(local);
            }
            assert_eq!(outcomes[0], outcomes[1]);
        }
    }
}
#[test]
fn two_bone_is_stable_with_fixed_target() {
    let rest = [
        BonePose::default(),
        BonePose {
            translation: Vec3::Y * 0.5,
            ..Default::default()
        },
        BonePose {
            translation: Vec3::Y * 0.5,
            ..Default::default()
        },
    ];
    let mut last = Quat::IDENTITY;
    for i in 0..300 {
        let mut local = rest;
        let mut sk = SkeletonView {
            parents: &[None, Some(0), Some(1)],
            rest: &rest,
            local: &mut local,
            root_global: Affine3A::IDENTITY,
        };
        process_two_bone(
            &mut sk,
            0,
            1,
            2,
            Vec3::new(0.5, 0.5, 0.1),
            Some(Vec3::Z),
            &TwoBoneSettings::default(),
        );
        if i > 0 {
            assert!(sk.local[0].rotation.dot(last).abs() > 1. - 1e-6)
        }
        last = sk.local[0].rotation;
    }
}

#[test]
fn virtual_end_from_parent_uses_the_rotated_rest_basis() {
    let rest = [
        BonePose::default(),
        BonePose {
            translation: Vec3::Y * 0.5,
            rotation: Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            ..Default::default()
        },
    ];
    let mut local = rest;
    let mut sk = SkeletonView {
        parents: &[None, Some(0)],
        rest: &rest,
        local: &mut local,
        root_global: Affine3A::IDENTITY,
    };
    let target = Vec3::new(0.6, 0.6, 0.);
    process_two_bone(
        &mut sk,
        0,
        1,
        1,
        target,
        Some(Vec3::Z),
        &TwoBoneSettings {
            use_virtual_end: true,
            extend_end_bone: true,
            end_bone_length: 0.5,
            ..Default::default()
        },
    );
    let local_axis = rest[1].rotation.inverse() * Vec3::Y;
    let effector = sk.skeleton_pose(1).transform_point3(local_axis * 0.5);
    assert!(effector.distance(target) < 1e-5, "{effector:?}");
}
