//! One rough core timing, excluding Bevy, rendering and setup allocations.
#![allow(dead_code, unused_imports)]
#[path = "../src/core/mod.rs"]
mod core;
use core::*;
use glam::{Affine3A, Vec3};
fn main() {
    let parents = [None, Some(0), Some(1), Some(2), Some(3), Some(4)];
    let rest = std::array::from_fn::<_, 6, _>(|i| BonePose {
        translation: if i == 0 { Vec3::ZERO } else { Vec3::Y * 0.5 },
        ..Default::default()
    });
    let mut springs = vec![SpringBoneState::default(); 75];
    let mut iterations = vec![IterateState::default(); 75];
    let start = std::time::Instant::now();
    for frame in 0..1000 {
        let t = frame as f32 / 60.;
        let goal = std::hint::black_box(Vec3::new(t.sin(), 1.8, 0.3));
        for i in 0..75 {
            let mut local = rest;
            let mut sk = SkeletonView { parents: &parents, rest: &rest,
                local: &mut local, root_global: Affine3A::IDENTITY };
            let chain = sk.chain(0, 5).unwrap();
            if i % 4 == 0 {
                process_spring_bones(&mut sk, &chain, &SpringBoneSettings::default(),
                    &mut springs[i], &[], Vec3::ZERO, 1. / 60.);
            } else {
                let solver = match i % 4 { 1 => IkSolver::Fabrik, 2 => IkSolver::Ccd,
                    _ => IkSolver::Jacobian };
                process_chain_ik(&mut sk, &chain, solver, &IterateSettings::default(),
                    &[], goal, &mut iterations[i]);
            }
            std::hint::black_box(local);
        }
    }
    let ms = start.elapsed().as_secs_f64() * 1000.;
    println!("75 six-joint chains × 1000 updates: {ms:.3} ms; {:.4} ms per batch", ms / 1000.);
}
