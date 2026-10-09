// Ported from godotengine/godot core/math/quaternion.cpp and basis.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
// Derived from Godot Engine core/math and SkeletonModifier3D; see LICENSE-MIT.
use glam::{Affine3A, Mat3, Quat, Vec3};
pub(crate) fn rotation(a: Affine3A) -> Quat {
    let mut x = vnorm(Vec3::from(a.matrix3.x_axis));
    let mut y = Vec3::from(a.matrix3.y_axis);
    y = vnorm(y - x * vdot(x, y));
    let mut z = Vec3::from(a.matrix3.z_axis);
    z = vnorm(z - x * vdot(x, z) - y * vdot(y, z));
    if x.length_squared() == 0. || y.length_squared() == 0. || z.length_squared() == 0. {
        return Quat::IDENTITY;
    }
    if x.dot(y.cross(z)) < 0. {
        x = -x;
        y = -y;
        z = -z;
    }
    matrix_quat(Mat3::from_cols(x, y, z))
}
pub(crate) fn arc(from: Vec3, to: Vec3) -> Quat {
    let a = vnorm(from);
    let b = vnorm(to);
    if a == Vec3::ZERO || b == Vec3::ZERO {
        return Quat::IDENTITY;
    }
    let d = vdot(a, b);
    if d.abs() > 0.99999975 {
        if d >= 0. {
            Quat::IDENTITY
        } else {
            let axis = a
                .cross(if a.x.abs() <= a.y.abs() && a.x.abs() <= a.z.abs() {
                    Vec3::X
                } else {
                    Vec3::Y
                })
                .normalize_or_zero();
            Quat::from_xyzw(axis.x, axis.y, axis.z, 0.)
        }
    } else {
        let c = vcross(a, b);
        let s = ((1. + d) * 2.).sqrt();
        qnorm(Quat::from_xyzw(
            c.x * (1. / s),
            c.y * (1. / s),
            c.z * (1. / s),
            s * 0.5,
        ))
    }
}
pub(crate) fn swing(q: Quat, axis: Vec3) -> Quat {
    let q = if (qdot(q, q) - 1.).abs() > 0.001 {
        qnorm(q)
    } else {
        q
    };
    let p = axis * vdot(Vec3::new(q.x, q.y, q.z), axis);
    let twist = Quat::from_xyzw(p.x, p.y, p.z, q.w);
    if qdot(twist, twist).abs() < 1e-5 {
        q
    } else {
        qnorm(qmul(
            q,
            if (qdot(twist, twist) - 1.).abs() > 0.001 {
                qnorm(twist)
            } else {
                twist
            }
            .inverse(),
        ))
    }
}
pub(crate) fn plane(normal: Vec3, v: Vec3) -> Vec3 {
    let n = normal.normalize_or_zero();
    v - n * v.dot(n)
}
pub(crate) fn limit_length(origin: Vec3, to: Vec3, len: f32) -> Vec3 {
    vadd_scaled(origin, vnorm(to - origin), len)
}
pub(crate) fn from_to(from: Vec3, to: Vec3, previous: Quat) -> Quat {
    let cross = from.cross(to);
    if (from.dot(to) + 1.).abs() < 1e-5 || cross.abs().max_element() < 1e-5 {
        return previous;
    }
    let angle = cross.length().atan2(from.dot(to));
    Quat::from_axis_angle(cross.normalize(), angle)
}
pub(crate) fn projected_normal(a: Vec3, b: Vec3, p: Vec3) -> Vec3 {
    let ab = b - a;
    let d = ab.length_squared();
    if d == 0. {
        Vec3::ZERO
    } else {
        (p - (a + ab * ((p - a).dot(ab) / d))).normalize_or_zero()
    }
}
pub(crate) fn slerp(a: Quat, b: Quat, t: f32) -> Quat {
    let mut b = b;
    let mut d = qdot(a, b);
    if d < 0. {
        d = -d;
        b = -b;
    }
    let (s0, s1) = if 1. - d > 1e-5 {
        let omega = d.clamp(-1., 1.).acos();
        let sine = omega.sin();
        (
            (((1. - t as f64) * omega as f64).sin() / sine as f64) as f32,
            (t * omega).sin() / sine,
        )
    } else {
        (1. - t, t)
    };
    Quat::from_xyzw(
        a.x * s0 + b.x * s1,
        a.y * s0 + b.y * s1,
        a.z * s0 + b.z * s1,
        a.w * s0 + b.w * s1,
    )
}
pub(crate) fn qx(q: Quat, v: Vec3) -> Vec3 {
    let u = Vec3::new(q.x, q.y, q.z);
    let uv = vcross(u, v);
    v + (uv * q.w + vcross(u, uv)) * 2.
}
pub(crate) fn qmul(a: Quat, b: Quat) -> Quat {
    Quat::from_xyzw(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
        a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )
}
pub(crate) fn qnorm(q: Quat) -> Quat {
    let r = 1. / qdot(q, q).sqrt();
    Quat::from_xyzw(q.x * r, q.y * r, q.z * r, q.w * r)
}
pub(crate) fn vnorm(v: Vec3) -> Vec3 {
    let l = vdot(v, v).sqrt();
    if l == 0. || !l.is_finite() {
        Vec3::ZERO
    } else {
        Vec3::new(v.x / l, v.y / l, v.z / l)
    }
}
pub(crate) fn vdot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}
pub(crate) fn vcross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}
pub(crate) fn qdot(a: Quat, b: Quat) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
}
pub(crate) fn vadd_scaled(a: Vec3, b: Vec3, s: f32) -> Vec3 {
    a + b * s
}

pub(crate) fn matrix_quat(m: Mat3) -> Quat {
    let rows = [
        Vec3::new(m.x_axis.x, m.y_axis.x, m.z_axis.x),
        Vec3::new(m.x_axis.y, m.y_axis.y, m.z_axis.y),
        Vec3::new(m.x_axis.z, m.y_axis.z, m.z_axis.z),
    ];
    let trace = rows[0][0] + rows[1][1] + rows[2][2];
    let mut q = [0.; 4];
    if trace > 0. {
        let s = (trace + 1.).sqrt();
        q[3] = s * 0.5;
        let r = 0.5 / s;
        q[0] = (rows[2][1] - rows[1][2]) * r;
        q[1] = (rows[0][2] - rows[2][0]) * r;
        q[2] = (rows[1][0] - rows[0][1]) * r;
    } else {
        let i = if rows[0][0] < rows[1][1] {
            if rows[1][1] < rows[2][2] { 2 } else { 1 }
        } else if rows[0][0] < rows[2][2] {
            2
        } else {
            0
        };
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;
        let s = (rows[i][i] - rows[j][j] - rows[k][k] + 1.).sqrt();
        q[i] = s * 0.5;
        let r = 0.5 / s;
        q[3] = (rows[k][j] - rows[j][k]) * r;
        q[j] = (rows[j][i] + rows[i][j]) * r;
        q[k] = (rows[k][i] + rows[i][k]) * r;
    }
    Quat::from_array(q)
}
pub(crate) fn pose_affine(scale: Vec3, q: Quat, t: Vec3) -> Affine3A {
    let d = qdot(q, q);
    if d < 1e-10 {
        return Affine3A::from_scale_rotation_translation(scale, Quat::IDENTITY, t);
    }
    let s = 2. / d;
    let xs = q.x * s;
    let ys = q.y * s;
    let zs = q.z * s;
    let wx = q.w * xs;
    let wy = q.w * ys;
    let wz = q.w * zs;
    let xx = q.x * xs;
    let xy = q.x * ys;
    let xz = q.x * zs;
    let yy = q.y * ys;
    let yz = q.y * zs;
    let zz = q.z * zs;
    Affine3A::from_mat3_translation(
        Mat3::from_cols(
            Vec3::new(1. - (yy + zz), xy + wz, xz - wy) * scale.x,
            Vec3::new(xy - wz, 1. - (xx + zz), yz + wx) * scale.y,
            Vec3::new(xz + wy, yz - wx, 1. - (xx + yy)) * scale.z,
        ),
        t,
    )
}
pub(crate) fn affine_mul(a: Affine3A, b: Affine3A) -> Affine3A {
    let x = Vec3::from(a.matrix3.x_axis);
    let y = Vec3::from(a.matrix3.y_axis);
    let z = Vec3::from(a.matrix3.z_axis);
    let rows = [
        Vec3::new(x.x, y.x, z.x),
        Vec3::new(x.y, y.y, z.y),
        Vec3::new(x.z, y.z, z.z),
    ];
    let xform = |v: Vec3| Vec3::new(vdot(rows[0], v), vdot(rows[1], v), vdot(rows[2], v));
    Affine3A::from_mat3_translation(
        Mat3::from_cols(
            xform(Vec3::from(b.matrix3.x_axis)),
            xform(Vec3::from(b.matrix3.y_axis)),
            xform(Vec3::from(b.matrix3.z_axis)),
        ),
        xform(Vec3::from(b.translation)) + Vec3::from(a.translation),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn godot_quat_from_two_vectors_matches_reference() {
        let rows: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/goldens/math.json")).unwrap();
        for row in rows.as_array().unwrap() {
            let v = |key: &str| {
                let a = row[key].as_array().unwrap();
                Vec3::new(
                    a[0].as_f64().unwrap() as f32,
                    a[1].as_f64().unwrap() as f32,
                    a[2].as_f64().unwrap() as f32,
                )
            };
            let a = row["quat"].as_array().unwrap();
            let q = Quat::from_xyzw(
                a[0].as_f64().unwrap() as f32,
                a[1].as_f64().unwrap() as f32,
                a[2].as_f64().unwrap() as f32,
                a[3].as_f64().unwrap() as f32,
            );
            assert!(arc(v("from"), v("to")).dot(q).abs() > 1. - 1e-6, "{row}");
        }
    }
}
