mod common;
use bevy_skeleton_modifiers::*;
use common::*;
use glam::{Affine3A, Quat, Vec3};
#[test]
fn spring_bone_scenarios_match_godot() {
    for name in [
        "spring_stiff",
        "spring_loose",
        "spring_gravity",
        "spring_sphere",
        "spring_capsule",
        "spring_inside_sphere",
        "spring_plane",
        "spring_hinge",
        "spring_center_bone",
        "spring_external",
    ] {
        let g = golden(name);
        let (parents, rest) = skeleton_data(&g);
        let mut state = SpringBoneState::default();
        let mut settings = SpringBoneSettings::default();
        if name == "spring_stiff" {
            settings.stiffness = 4.
        }
        if name == "spring_loose" {
            settings.drag = 0.1
        }
        if name == "spring_gravity" {
            settings.gravity = 0.5
        }
        if name == "spring_hinge" {
            settings.rotation_axis = RotationAxis::Z
        }
        if name == "spring_center_bone" {
            settings.center_from = SpringCenter::Bone(0)
        }
        for (i, frame) in g["frames"].as_array().unwrap().iter().enumerate() {
            let mut local: Vec<_> = frame["input"]
                .as_array()
                .unwrap()
                .iter()
                .map(pose)
                .collect();
            let root = affine(&frame["root"]);
            let mut sk = SkeletonView {
                parents: &parents,
                rest: &rest,
                local: &mut local,
                root_global: root,
            };
            let chain = sk.chain(0, 4).unwrap();
            let collider = match name {
                "spring_sphere" => Some(SpringCollider::Sphere {
                    radius: 0.35,
                    inside: false,
                }),
                "spring_inside_sphere" => Some(SpringCollider::Sphere {
                    radius: 1.6,
                    inside: true,
                }),
                "spring_capsule" => Some(SpringCollider::Capsule {
                    radius: 0.35,
                    height: 1.1,
                    inside: false,
                }),
                "spring_plane" => Some(SpringCollider::Plane),
                _ => None,
            };
            let mut cols = Vec::new();
            if let Some(c) = collider {
                let p = if name == "spring_plane" {
                    Affine3A::from_rotation_translation(
                        Quat::from_rotation_z(0.45),
                        Vec3::new(0., -0.1, 0.),
                    )
                } else {
                    Affine3A::from_translation(Vec3::new(0.2, 1.1, 0.05))
                };
                cols.push((c, root * p));
            }
            let external = if name == "spring_external" {
                Vec3::new(0.7, 0.1, 0.3)
            } else {
                Vec3::ZERO
            };
            process_spring_bones(
                &mut sk,
                &chain,
                &settings,
                &mut state,
                &cols,
                external,
                number(&frame["delta"]),
            );
            compare(&sk, frame, name, i, 1e-3);
        }
    }
}
#[test]
fn spring_bone_dt_extremes_stay_finite() {
    let rest = [
        BonePose::default(),
        BonePose {
            translation: Vec3::Y,
            ..Default::default()
        },
    ];
    for dt in [0., -0.1, 0.5, f32::NAN] {
        let mut local = rest;
        let mut sk = SkeletonView {
            parents: &[None, Some(0)],
            rest: &rest,
            local: &mut local,
            root_global: Affine3A::IDENTITY,
        };
        let chain = sk.chain(0, 1).unwrap();
        process_spring_bones(
            &mut sk,
            &chain,
            &SpringBoneSettings::default(),
            &mut SpringBoneState::default(),
            &[],
            Vec3::X,
            dt,
        );
        assert!(sk.local.iter().all(|p| p.rotation.is_finite()));
        if dt <= 0. || !dt.is_finite() {
            assert_eq!(sk.local, &rest)
        }
    }
}
#[test]
fn spring_bone_settles_when_root_still() {
    let rest = [
        BonePose::default(),
        BonePose {
            translation: Vec3::Y,
            ..Default::default()
        },
    ];
    let mut state = SpringBoneState::default();
    let mut last = Quat::IDENTITY;
    for i in 0..400 {
        let mut local = rest;
        let mut sk = SkeletonView {
            parents: &[None, Some(0)],
            rest: &rest,
            local: &mut local,
            root_global: Affine3A::IDENTITY,
        };
        let chain = sk.chain(0, 1).unwrap();
        process_spring_bones(
            &mut sk,
            &chain,
            &SpringBoneSettings::default(),
            &mut state,
            &[],
            if i < 30 { Vec3::X } else { Vec3::ZERO },
            1. / 60.,
        );
        if i > 300 {
            assert!(sk.local[0].rotation.dot(last).abs() > 1. - 1e-6)
        }
        last = sk.local[0].rotation;
    }
}
#[test]
fn spring_response_is_invariant_to_uniform_root_scale() {
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
    let mut states = [
        SpringBoneState::default(),
        SpringBoneState::default(),
        SpringBoneState::default(),
    ];
    for frame in 0..300 {
        let t = frame as f32 / 60.;
        let mut output = Vec::new();
        for (scale, state) in [0.01, 1., 2.].into_iter().zip(&mut states) {
            let mut local = rest;
            let mut sk = SkeletonView {
                parents: &[None, Some(0), Some(1)],
                rest: &rest,
                local: &mut local,
                root_global: Affine3A::from_scale_rotation_translation(
                    Vec3::splat(scale),
                    Quat::from_rotation_z(t.sin() * 0.1),
                    Vec3::new(t.sin() * 0.2, 0., 0.) * scale,
                ),
            };
            let chain = sk.chain(0, 2).unwrap();
            process_spring_bones(
                &mut sk,
                &chain,
                &SpringBoneSettings::default(),
                state,
                &[],
                Vec3::ZERO,
                1. / 60.,
            );
            output.push(local);
        }
        for ((small, unit), large) in output[0].iter().zip(&output[1]).zip(&output[2]) {
            assert!(small.rotation.dot(unit.rotation).abs() > 1. - 1e-6);
            assert!(large.rotation.dot(unit.rotation).abs() > 1. - 1e-6);
        }
    }
}
