mod common;
use bevy_skeleton_modifiers::*;
use common::*;
use glam::{Affine3A, Quat, Vec3};
#[test]
fn fabrik_ccd_jacobian_match_godot() {
    for (prefix, solver) in [
        ("fabrik", IkSolver::Fabrik),
        ("ccd", IkSolver::Ccd),
        ("jacobian", IkSolver::Jacobian),
    ] {
        for suffix in ["free", "limits", "warm"] {
            let name = format!("{prefix}_{suffix}");
            let g = golden(&name);
            let (parents, rest) = skeleton_data(&g);
            let mut state = IterateState::default();
            let mut worst = (0_f32, 0_f32);
            let settings = IterateSettings {
                deterministic: suffix != "warm",
                ..Default::default()
            };
            let mut limits = [None; 6];
            if suffix == "limits" {
                for l in &mut limits[2..5] {
                    *l = Some(JointLimit::Cone {
                        angle: 0.45,
                        right_axis: SecondaryDirection::None,
                        right_axis_vector: Vec3::X,
                        rotation_offset: Quat::IDENTITY,
                    });
                }
            }
            for (i, f) in g["frames"].as_array().unwrap().iter().enumerate() {
                let mut local: Vec<_> = f["input"].as_array().unwrap().iter().map(pose).collect();
                let mut sk = SkeletonView {
                    parents: &parents,
                    rest: &rest,
                    local: &mut local,
                    root_global: affine(&f["root"]),
                };
                let chain = sk.chain(0, 5).unwrap();
                process_chain_ik(
                    &mut sk,
                    &chain,
                    solver,
                    &settings,
                    &limits,
                    vec(&f["target"]),
                    &mut state,
                );
                let error = compare(
                    &sk,
                    f,
                    &name,
                    i,
                    if name == "ccd_limits" { 1e-2 } else { 1e-3 },
                );
                worst.0 = worst.0.max(error.0);
                worst.1 = worst.1.max(error.1);
            }
            println!(
                "{name}: worst position {}, rotation {} rad",
                worst.0, worst.1
            );
        }
    }
}
#[test]
fn cone_limit_never_exceeded() {
    let limit = JointLimit::Cone {
        angle: 0.6,
        right_axis: SecondaryDirection::None,
        right_axis_vector: Vec3::X,
        rotation_offset: Quat::IDENTITY,
    };
    for i in 0..1000 {
        let a = i as f32 * 0.03;
        let input = Vec3::new(a.sin(), a.cos(), (a * 0.7).sin()).normalize();
        let output = limit.solve(Vec3::Y, input).normalize();
        assert!(output.dot(Vec3::Y).clamp(-1., 1.).acos() <= 0.3 + 1e-4);
    }
}
#[test]
fn chain_ik_degenerate_inputs_are_finite() {
    for solver in [IkSolver::Fabrik, IkSolver::Ccd, IkSolver::Jacobian] {
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
            let mut state = IterateState::default();
            for target in [Vec3::ZERO, Vec3::Y * 100., Vec3::Y * 0.1] {
                let mut local = rest;
                let mut sk = SkeletonView {
                    parents: &[None, Some(0), Some(1)],
                    rest: &rest,
                    local: &mut local,
                    root_global: Affine3A::IDENTITY,
                };
                let chain = sk.chain(0, 2).unwrap();
                process_chain_ik(
                    &mut sk,
                    &chain,
                    solver,
                    &IterateSettings::default(),
                    &[],
                    target,
                    &mut state,
                );
                assert!(sk.local.iter().all(|p| p.rotation.is_finite()));
            }
        }
    }
}
#[test]
fn chain_ik_is_stable_with_fixed_target() {
    for solver in [IkSolver::Fabrik, IkSolver::Ccd, IkSolver::Jacobian] {
        for deterministic in [true, false] {
            for target in [Vec3::new(3., 3., 1.), Vec3::new(0.6, 0.8, 0.2)] {
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
                    BonePose {
                        translation: Vec3::Y * 0.5,
                        ..Default::default()
                    },
                ];
                let mut state = IterateState::default();
                let mut last = [Quat::IDENTITY; 4];
                for i in 0..350 {
                    let mut local = rest;
                    let mut sk = SkeletonView {
                        parents: &[None, Some(0), Some(1), Some(2)],
                        rest: &rest,
                        local: &mut local,
                        root_global: Affine3A::IDENTITY,
                    };
                    let chain = sk.chain(0, 3).unwrap();
                    process_chain_ik(
                        &mut sk,
                        &chain,
                        solver,
                        &IterateSettings {
                            deterministic,
                            ..Default::default()
                        },
                        &[],
                        target,
                        &mut state,
                    );
                    for (j, p) in sk.local.iter().enumerate() {
                        if i > 300 {
                            let delta = last[j].inverse() * p.rotation;
                            let radians = 2.
                                * Vec3::new(delta.x, delta.y, delta.z)
                                    .length()
                                    .atan2(delta.w.abs());
                            assert!(
                                radians <= 1e-5,
                                "{solver:?} {deterministic} frame {i} joint {j} delta {radians}"
                            )
                        }
                        last[j] = p.rotation;
                    }
                }
            }
        }
    }
}
