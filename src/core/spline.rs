// Ported from godotengine/godot scene/3d/spline_ik_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Baked-path spline IK derived from Godot SplineIK3D.
use super::{ChainIndices, SkeletonView, godot_math::*};
use glam::{Affine3A, Quat, Vec3};
/// A path baked during setup. Point and tilt arrays must have the same length.
#[derive(Clone, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_splineik3d.html).
pub struct BakedPath {
    /// Baked path positions in path coordinates.
    pub points: Vec<Vec3>,
    /// Roll angles in radians, one per baked point.
    pub tilts: Vec<f32>,
    /// Requested arc-length spacing used when baking.
    pub interval: f32,
}
impl BakedPath {
    /// Resample a finite-domain Bevy curve at approximately uniform arc length.
    /// Invalid domains/intervals produce an empty path. The final segment may be shorter.
    #[cfg(feature = "bevy")]
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_splineik3d.html).
    pub fn from_curve(curve: &impl bevy_math::curve::Curve<Vec3>, interval: f32) -> Self {
        let domain = curve.domain();
        let mut result = Self {
            points: Vec::new(),
            tilts: Vec::new(),
            interval,
        };
        let start = domain.start();
        let end = domain.end();
        if !start.is_finite()
            || !end.is_finite()
            || !interval.is_finite()
            || interval <= 0.
            || end <= start
        {
            return result;
        }
        let Some(mut previous) = curve.sample(start) else {
            return result;
        };
        if !previous.is_finite() {
            return result;
        }
        result.points.push(previous);
        let mut remaining = interval;
        for i in 1..=16384 {
            let Some(point) = curve.sample(start + (end - start) * (i as f32 / 16384.)) else {
                continue;
            };
            if !point.is_finite() {
                return Self {
                    points: Vec::new(),
                    tilts: Vec::new(),
                    interval,
                };
            }
            let mut segment = point - previous;
            let mut length = segment.length();
            while length >= remaining {
                let next = previous + segment * (remaining / length);
                result.points.push(next);
                previous = next;
                segment = point - previous;
                length = segment.length();
                remaining = interval;
            }
            remaining -= length;
            previous = point;
        }
        if result
            .points
            .last()
            .is_some_and(|p| p.distance(previous) > 1e-6)
        {
            result.points.push(previous)
        }
        result.tilts.resize(result.points.len(), 0.);
        result
    }
}
/// Tilt interpolation at the beginning and end of a path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_splineik3d.html).
pub struct SplineSettings {
    /// Apply the baked roll angles.
    pub tilt_enabled: bool,
    /// Number of joints for the initial fade; zero holds tilt, negative disables it.
    pub tilt_fade_in: i32,
    /// Number of joints for the final fade; zero holds tilt, negative disables it.
    pub tilt_fade_out: i32,
}
impl Default for SplineSettings {
    fn default() -> Self {
        Self {
            tilt_enabled: true,
            tilt_fade_in: 1,
            tilt_fade_out: 1,
        }
    }
}
fn nearest(origin: Vec3, length_sq: f64, points: &[Vec3], first: usize) -> (usize, f64) {
    let mut i = first.min(points.len() - 1);
    while i < points.len() {
        if origin.distance_squared(points[i]) as f64 >= length_sq {
            break;
        }
        i += 1;
    }
    if i == 0 {
        (first, 0.)
    } else if i >= points.len() {
        (i, 1.)
    } else {
        let a = origin.distance_squared(points[i - 1]) as f64;
        let b = origin.distance_squared(points[i]) as f64;
        (
            i - 1,
            if b == a {
                0.
            } else {
                (length_sq - a) / (b - a)
            },
        )
    }
}
/// Orient a chain along an open baked path, without update-time allocation.
/// `path_to_skeleton` places the path in skeleton coordinates. The pinned Godot
/// open-path sample selection is preserved; see PORTING.md for this upstream quirk.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_splineik3d.html).
pub fn process_spline_ik(
    sk: &mut SkeletonView<'_>,
    chain: &ChainIndices,
    path: &BakedPath,
    path_to_skeleton: Affine3A,
    settings: &SplineSettings,
) {
    if sk.validate().is_err()
        || chain.len() < 2
        || path.points.is_empty()
        || path.tilts.len() != path.points.len()
        || path.points.iter().any(|p| !p.is_finite())
        || path.tilts.iter().any(|t| !t.is_finite())
        || chain
            .as_slice()
            .iter()
            .any(|&i| i as usize >= sk.local.len())
    {
        return;
    }
    let n = chain.len();
    let mut points = [Vec3::ZERO; 32];
    let mut forward = [Vec3::ZERO; 32];
    let mut lengths = [0.; 32];
    let mut accum = [0_f64; 32];
    let mut twists = [0_f64; 32];
    let mut total = 0.;
    for (i, &bone) in chain.as_slice().iter().enumerate() {
        points[i] = Vec3::from(sk.skeleton_pose(bone).translation);
        if i + 1 < n {
            let axis = sk.local[chain.as_slice()[i + 1] as usize].translation;
            forward[i] = axis.normalize_or_zero();
            lengths[i] = axis.length();
            total += lengths[i] as f64;
        }
        accum[i] = total;
    }
    let start_point = path_to_skeleton.transform_point3(path.points[0]);
    let start_vector = start_point - points[0];
    let start_dist = start_vector.length() as f64;
    let chain_start = accum[..n]
        .iter()
        .position(|&l| l >= start_dist)
        .unwrap_or(n - 1);
    let denom_start = if settings.tilt_fade_in > 0 {
        (settings.tilt_fade_in - 1).clamp(chain_start as i32, n as i32)
    } else {
        -1
    };
    let mut fade_in = 0.;
    for i in ((denom_start - settings.tilt_fade_in + 1).max(0)..=denom_start).rev() {
        fade_in += lengths.get(i as usize).copied().unwrap_or(0.) as f64;
    }
    let point_last = path.points.len() - 1;
    let end_point = path_to_skeleton.transform_point3(path.points[point_last]);
    let mut last_nearest = 0;
    let mut last_next = 0;
    let mut last_t = 0.;
    let mut ended = 0;
    let mut end_vector = Vec3::ZERO;
    let mut end_length = 0.;
    let mut fade_out = 0.;
    let mut i = 0;
    while i < n - 1 {
        let len = lengths[i];
        if len < 1e-5 {
            i += 1;
            continue;
        }
        let fitting = i == chain_start;
        if ended > 0 {
            points[i + 1] = limit_length(points[i], points[i] + end_vector, len);
            if settings.tilt_enabled {
                twists[i] = if settings.tilt_fade_out < 0 {
                    0.
                } else if settings.tilt_fade_out == 0 {
                    path.tilts[point_last] as f64
                } else {
                    if ended == 1 {
                        end_length = points[i + 1].distance(end_point) as f64
                    } else {
                        end_length += len as f64
                    }
                    let damping = if fade_out == 0. {
                        1.
                    } else {
                        (end_length / fade_out).clamp(0., 1.)
                    };
                    let initial = if ended == 1 {
                        path.tilts[last_nearest] as f64
                            + (path.tilts[last_next] as f64 - path.tilts[last_nearest] as f64)
                                * last_t
                    } else {
                        path.tilts[point_last] as f64
                    };
                    ended = 2;
                    initial * (1. - damping)
                };
            }
            i += 1;
            continue;
        }
        if path.points.len() == 1 || i <= chain_start {
            let update = !fitting || path.points.len() == 1;
            if update {
                points[i + 1] = limit_length(points[i], points[i] + start_vector, len)
            }
            if settings.tilt_enabled {
                twists[i] = if settings.tilt_fade_in < 0 {
                    0.
                } else if settings.tilt_fade_in == 0 {
                    path.tilts[0] as f64
                } else {
                    let damping = if fade_in == 0. {
                        1.
                    } else {
                        (points[i].distance(start_point) as f64 / fade_in).clamp(0., 1.)
                    };
                    path.tilts[0] as f64 * (1. - damping)
                };
            }
            if update {
                i += 1;
                continue;
            }
        }
        let (mut sample, mut t) = nearest(
            path_to_skeleton.inverse().transform_point3(points[i]),
            (len * len) as f64,
            &path.points,
            last_nearest,
        );
        if sample >= path.points.len() {
            if i == 0 {
                sample = path.points.len().saturating_sub(2);
                t = 1.;
            } else {
                let chain_end = (points[i] - points[i - 1]).normalize_or_zero();
                let path_end = (end_point - points[i]).normalize_or_zero();
                let mut distance_to_last = 0_f64;
                let mut path_length = 0_f64;
                for j in 1..path.points.len() {
                    let d = path.points[j].distance(path.points[j - 1]) as f64;
                    path_length += d;
                    if j <= last_nearest {
                        distance_to_last += d;
                    }
                }
                let next_dist = if last_next > last_nearest {
                    path.points[last_next].distance(path.points[last_nearest]) as f64
                } else {
                    0.
                };
                let rest_length = path_length - distance_to_last - next_dist * last_t;
                let t = (rest_length / len as f64).clamp(0., 1.) as f32;
                end_vector = chain_end.lerp(path_end, t);
                if settings.tilt_fade_out > 0 {
                    let start =
                        (n as i32 - 1 - settings.tilt_fade_out).clamp(0, last_nearest as i32);
                    for e in start..start + settings.tilt_fade_out {
                        fade_out += lengths.get(e as usize).copied().unwrap_or(0.) as f64;
                    }
                }
                ended = 1;
                continue;
            }
        }
        // Godot 4.7.2 intentionally retained: open-path next is the same sample.
        let next = sample.min(point_last);
        points[i + 1] = limit_length(
            points[i],
            path_to_skeleton
                .transform_point3(path.points[sample].lerp(path.points[next], t as f32)),
            len,
        );
        if !fitting {
            twists[i] = path.tilts[last_nearest] as f64
                + (path.tilts[last_next] as f64 - path.tilts[last_nearest] as f64) * last_t;
        }
        last_nearest = sample;
        last_next = next;
        last_t = t;
        i += 1;
    }
    let mut parent = sk.parents[chain.as_slice()[0] as usize]
        .map_or(Quat::IDENTITY, |p| rotation(sk.skeleton_pose(p)));
    let mut parent_twist = 0.;
    for i in 0..n - 1 {
        if lengths[i] < 1e-5 {
            continue;
        }
        let bone = chain.as_slice()[i];
        let input = rotation(sk.local[bone as usize].affine());
        let grest = (parent * input).normalize();
        let to =
            (grest.inverse() * (points[i + 1] - points[i]).normalize_or_zero()).normalize_or_zero();
        let mut q = swing(arc(forward[i], to), forward[i]);
        if settings.tilt_enabled {
            q *= Quat::from_axis_angle(forward[i], (twists[i] - parent_twist) as f32);
            parent_twist = twists[i];
        }
        let local = (input * q).normalize();
        if local.is_finite() {
            sk.local[bone as usize].rotation = local;
            parent = (parent * local).normalize();
        }
    }
}
