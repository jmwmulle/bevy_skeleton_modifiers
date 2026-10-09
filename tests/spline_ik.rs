mod common;
use bevy_skeleton_modifiers::*;
use common::*;
#[test]
fn spline_ik_matches_godot_on_godot_baked_points() {
    for name in ["spline_plain", "spline_tilt"] {
        let g = golden(name);
        let (parents, rest) = skeleton_data(&g);
        let path = BakedPath {
            points: g["points"].as_array().unwrap().iter().map(vec).collect(),
            tilts: g["tilts"].as_array().unwrap().iter().map(number).collect(),
            interval: 0.05,
        };
        for (i, f) in g["frames"].as_array().unwrap().iter().enumerate() {
            let mut local: Vec<_> = f["input"].as_array().unwrap().iter().map(pose).collect();
            let root = affine(&f["root"]);
            let mut sk = SkeletonView {
                parents: &parents,
                rest: &rest,
                local: &mut local,
                root_global: root,
            };
            let chain = sk.chain(0, 5).unwrap();
            process_spline_ik(
                &mut sk,
                &chain,
                &path,
                root.inverse(),
                &SplineSettings {
                    tilt_enabled: name == "spline_tilt",
                    ..Default::default()
                },
            );
            compare(&sk, f, name, i, 1e-3);
        }
    }
}
#[cfg(feature = "bevy")]
#[test]
fn bake_from_bevy_curve_is_arc_length_uniform() {
    use bevy_math::curve::{FunctionCurve, Interval};
    use glam::Vec3;
    let curve = FunctionCurve::new(Interval::new(0., 6.).unwrap(), |t| {
        Vec3::new(t.cos(), t.sin(), t * 0.2)
    });
    let path = BakedPath::from_curve(&curve, 0.05);
    assert!(path.points.len() > 100);
    for pair in path.points.windows(2).take(path.points.len() - 2) {
        assert!((pair[0].distance(pair[1]) / 0.05 - 1.).abs() < 0.02)
    }
}
