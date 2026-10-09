#![allow(dead_code)]
use bevy_skeleton_modifiers::*;
use glam::{Affine3A, Quat, Vec3};
use serde_json::Value;
use std::io::Read;
pub fn number(v: &Value) -> f32 {
    v.as_f64().unwrap() as f32
}
pub fn vec(v: &Value) -> Vec3 {
    Vec3::new(number(&v[0]), number(&v[1]), number(&v[2]))
}
pub fn pose(v: &Value) -> BonePose {
    BonePose {
        translation: vec(v),
        rotation: Quat::from_xyzw(number(&v[3]), number(&v[4]), number(&v[5]), number(&v[6])),
        scale: Vec3::new(number(&v[7]), number(&v[8]), number(&v[9])),
    }
}
pub fn golden(name: &str) -> Value {
    let folder = std::env::var("SKELETON_GOLDENS")
        .unwrap_or_else(|_| format!("{}/tests/goldens", env!("CARGO_MANIFEST_DIR")));
    let raw = std::fs::read(format!("{folder}/{name}.json.gz")).unwrap();
    let mut decoded = String::new();
    flate2::read::GzDecoder::new(&raw[..])
        .read_to_string(&mut decoded)
        .unwrap();
    serde_json::from_str(&decoded).unwrap()
}
pub fn skeleton_data(g: &Value) -> (Vec<Option<u16>>, Vec<BonePose>) {
    (
        g["parents"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| p.as_u64().map(|i| i as u16))
            .collect(),
        g["rest"].as_array().unwrap().iter().map(pose).collect(),
    )
}
pub fn compare(
    sk: &SkeletonView<'_>,
    frame: &Value,
    name: &str,
    index: usize,
    tolerance: f32,
) -> (f32, f32) {
    let mut worst_position = 0_f32;
    let mut worst_angle = 0_f32;
    for i in 0..sk.local.len() {
        let world = sk.global_pose(i as u16);
        let row = &frame["output"][i];
        let expected_pos = vec(row);
        let (_, actual_rot, _) = world.to_scale_rotation_translation();
        let expected_rot = Quat::from_xyzw(
            number(&row[3]),
            number(&row[4]),
            number(&row[5]),
            number(&row[6]),
        );
        let position = Vec3::from(world.translation).distance(expected_pos);
        let relative = actual_rot.normalize().inverse() * expected_rot.normalize();
        let angle = 2.
            * Vec3::new(relative.x, relative.y, relative.z)
                .length()
                .atan2(relative.w.abs());
        worst_position = worst_position.max(position);
        worst_angle = worst_angle.max(angle);
        assert!(
            position
                <= tolerance
                    * if name.starts_with("fabrik")
                        || name.starts_with("ccd")
                        || name.starts_with("jacobian")
                    {
                        1.
                    } else {
                        0.5
                    }
                && angle <= tolerance,
            "{name}, frame {index}, bone {i}: position {position}, angle {angle}, actual {:?}, reference {:?}",
            actual_rot,
            expected_rot
        )
    }
    (worst_position, worst_angle)
}

pub fn affine(v: &Value) -> Affine3A {
    pose(v).affine()
}
