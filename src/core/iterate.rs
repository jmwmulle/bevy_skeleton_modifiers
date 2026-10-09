// Ported from godotengine/godot scene/3d/{iterate_ik,fabr_ik,ccd_ik,jacobian_ik}_3d.cpp
// at commit ed1daf0bf001b61586d9930840f2f1394092c079.
// Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
// Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: translated from C++ to Rust; borrowed poses and fixed-capacity state.
// See PORTING.md for intentional behavioral changes.
//! Iterative IK translated from Godot's ChainIK3D and IterateIK3D families.
use super::{ChainIndices, JointLimit, SkeletonView, godot_math::*};
use glam::{Quat, Vec3};
/// Iteration and continuity controls; defaults match Godot.
#[derive(Clone, Copy, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_iterateik3d.html).
pub struct IterateSettings {
    /// Maximum solver passes per update.
    pub max_iterations: u32,
    /// Endpoint distance at which iteration stops.
    pub min_distance: f64,
    /// Maximum angular change per pass, in radians.
    pub angular_delta_limit: f64,
    /// Start each update from the input pose rather than stored coordinates.
    pub deterministic: bool,
}
impl Default for IterateSettings {
    fn default() -> Self {
        Self {
            max_iterations: 4,
            min_distance: 0.001,
            angular_delta_limit: 2_f64.to_radians(),
            deterministic: false,
        }
    }
}
/// Chain solving algorithm.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_iterateik3d.html).
pub enum IkSolver {
    /// Forward and backward reaching.
    Fabrik,
    /// Cyclic coordinate descent.
    Ccd,
    /// Jacobian-transpose rotation steps.
    Jacobian,
}
#[derive(Clone, Copy, Debug)]
struct Joint {
    forward: Vec3,
    len: f32,
    vector: Vec3,
    grest: Quat,
    local: Quat,
    global: Quat,
}
impl Default for Joint {
    fn default() -> Self {
        Self {
            forward: Vec3::ZERO,
            len: 0.,
            vector: Vec3::ZERO,
            grest: Quat::IDENTITY,
            local: Quat::IDENTITY,
            global: Quat::IDENTITY,
        }
    }
}
/// Fixed-capacity warm-start storage; reset after replacing a chain.
#[derive(Clone, Debug)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_iterateik3d.html).
pub struct IterateState {
    points: [Vec3; 32],
    joints: [Joint; 32],
    initialized: bool,
    n: usize,
}
impl Default for IterateState {
    fn default() -> Self {
        Self {
            points: [Vec3::ZERO; 32],
            joints: [Joint::default(); 32],
            initialized: false,
            n: 0,
        }
    }
}
impl IterateState {
    /// Restart from the next input pose.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_iterateik3d.html).
    pub fn reset(&mut self) {
        self.initialized = false;
    }
    fn vectors(&mut self, index: usize) {
        if index > 0 {
            self.joints[index - 1].vector = vnorm(self.points[index] - self.points[index - 1])
        }
        if index + 1 < self.n {
            self.joints[index].vector = vnorm(self.points[index + 1] - self.points[index])
        }
    }
    fn update(&mut self, index: usize, point: Vec3, backwards: bool) {
        if self.points[index].distance(point) < 1e-5 {
            return;
        }
        if backwards && index > 0 {
            let j = self.joints[index - 1];
            let new = vnorm(point - self.points[index - 1]);
            if (vdot(j.vector, new) + 1.).abs() < 1e-5 {
                self.points[index] = self.points[index - 1] + j.vector * j.len;
                return;
            }
        } else if !backwards && index + 1 < self.n {
            let j = self.joints[index];
            let new = vnorm(self.points[index + 1] - point);
            if (vdot(j.vector, new) + 1.).abs() < 1e-5 {
                self.points[index] = self.points[index + 1] - j.vector * j.len;
                return;
            }
        }
        self.points[index] = point;
        self.vectors(index);
    }
    fn cache(&mut self, sk: &SkeletonView<'_>, chain: &ChainIndices, angular_limit: f64) {
        let mut parent = sk.parents[chain.as_slice()[0] as usize]
            .map_or(Quat::IDENTITY, |p| rotation(sk.skeleton_pose(p)));
        for i in 0..self.n - 1 {
            let input = rotation(sk.local[chain.as_slice()[i] as usize].affine());
            let j = &mut self.joints[i];
            j.grest = qnorm(qmul(parent, input));
            let next = qmul(
                input,
                swing(
                    arc(j.forward, vnorm(qx(j.grest.inverse(), j.vector))),
                    j.forward,
                ),
            );
            let diff = (qdot(j.local, next).powi(2) * 2. - 1.)
                .clamp(-1., 1.)
                .acos();
            j.local = if diff > 1e-5 {
                slerp(
                    j.local,
                    next,
                    ((angular_limit / diff as f64).min(1.)) as f32,
                )
            } else {
                next
            };
            j.global = qnorm(qmul(parent, j.local));
            parent = j.global;
        }
        self.points[0] = Vec3::from(sk.skeleton_pose(chain.as_slice()[0]).translation);
        for i in 0..self.n - 1 {
            self.points[i + 1] = vadd_scaled(
                self.points[i],
                qx(self.joints[i].global, self.joints[i].forward),
                self.joints[i].len,
            );
        }
        for i in 0..self.n {
            self.vectors(i)
        }
    }
    fn constrained(
        &mut self,
        sk: &SkeletonView<'_>,
        index: usize,
        head: usize,
        tail: usize,
        backwards: bool,
        limits: &[Option<JointLimit>],
        basis: usize,
    ) {
        if let Some(Some(limit)) = limits.get(index) {
            let j = self.joints[basis];
            let vector = self.points[tail] - self.points[head];
            let local = qx(j.grest.inverse(), vector);
            let length = local.length();
            if length < 1e-5 {
                return;
            }
            let limited = qx(
                j.grest,
                limit.solve_direction(j.forward, vnorm(local)) * length,
            );
            self.update(tail, self.points[head] + limited, backwards)
        }
        let _ = sk;
    }
}
/// Solve a validated chain toward a skeleton-space target without allocating.
/// Persistent state is used only when `deterministic` is false.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_iterateik3d.html).
pub fn process_chain_ik(
    sk: &mut SkeletonView<'_>,
    chain: &ChainIndices,
    solver: IkSolver,
    settings: &IterateSettings,
    limits: &[Option<JointLimit>],
    target: Vec3,
    state: &mut IterateState,
) {
    if sk.validate().is_err()
        || chain.len() < 2
        || chain
            .as_slice()
            .iter()
            .any(|&i| i as usize >= sk.local.len())
        || !target.is_finite()
    {
        return;
    }
    let n = chain.len();
    let root = Vec3::from(sk.skeleton_pose(chain.as_slice()[0]).translation);
    if !state.initialized || state.n != n || settings.deterministic {
        state.n = n;
        for (i, &bone) in chain.as_slice().iter().enumerate() {
            state.points[i] = Vec3::from(sk.skeleton_pose(bone).translation);
            if i + 1 < n {
                let axis = sk.local[chain.as_slice()[i + 1] as usize].translation;
                state.joints[i] = Joint {
                    forward: axis.normalize_or_zero(),
                    len: axis.length(),
                    local: rotation(sk.local[bone as usize].affine()),
                    global: rotation(sk.skeleton_pose(bone)),
                    ..Default::default()
                };
            }
        }
        for i in 0..n {
            state.vectors(i)
        }
        state.initialized = true;
    }
    for i in 0..n - 1 {
        let axis = sk.local[chain.as_slice()[i + 1] as usize].translation;
        state.joints[i].forward = axis.normalize_or_zero();
        state.joints[i].len = axis.length();
    }
    state.cache(sk, chain, std::f64::consts::PI);
    let angular = if settings.angular_delta_limit.is_finite() {
        settings.angular_delta_limit.max(0.)
    } else {
        0.
    };
    let total_length: f32 = state.joints[..n - 1].iter().map(|j| j.len).sum();
    let unreachable =
        (target - root).length() >= total_length && limits.iter().all(Option::is_none);
    for _ in 0..settings.max_iterations {
        if unreachable {
            let direction = vnorm(target - root);
            state.points[0] = root;
            for i in 0..n - 1 {
                state.points[i + 1] = state.points[i] + direction * state.joints[i].len;
            }
            for i in 0..n {
                state.vectors(i);
            }
        } else {
            match solver {
                IkSolver::Fabrik => {
                    let mut first = true;
                    for i in (0..n - 1).rev() {
                        if state.joints[i].len < 1e-5 {
                            continue;
                        }
                        if first {
                            state.update(i + 1, target, true);
                            first = false;
                        }
                        let head =
                            limit_length(state.points[i + 1], state.points[i], state.joints[i].len);
                        state.update(i, head, true);
                        state.constrained(sk, i, i + 1, i, true, limits, i);
                    }
                    first = true;
                    for i in 0..n - 1 {
                        if state.joints[i].len < 1e-5 {
                            continue;
                        }
                        if first {
                            state.update(i, root, false);
                            first = false;
                        }
                        let tail =
                            limit_length(state.points[i], state.points[i + 1], state.joints[i].len);
                        state.update(i + 1, tail, false);
                        state.constrained(sk, i, i, i + 1, false, limits, i);
                    }
                }
                IkSolver::Ccd => {
                    for ancestor in (0..n).rev() {
                        for i in ancestor..n - 1 {
                            if state.joints[i].len < 1e-5 {
                                continue;
                            }
                            let head = state.points[i];
                            let to_effector = state.points[n - 1] - head;
                            let to_target = target - head;
                            if to_effector.length_squared() * to_target.length_squared() < 1e-5 {
                                continue;
                            }
                            let q = arc(vnorm(to_effector), vnorm(to_target));
                            state.update(i + 1, head + qx(q, state.points[i + 1] - head), false);
                            state.constrained(sk, i, i, i + 1, false, limits, i);
                        }
                    }
                }
                IkSolver::Jacobian => {
                    for i in 0..n - 1 {
                        if state.joints[i].len < 1e-5 {
                            continue;
                        }
                        let head = state.points[i];
                        let to_effector = state.points[n - 1] - head;
                        let to_target = target - state.points[n - 1];
                        let axis = to_effector.cross(to_target);
                        if axis.length_squared() < 1e-5 {
                            continue;
                        }
                        let q = Quat::from_axis_angle(
                            vnorm(axis),
                            ((axis.length() as f64
                                / (to_effector.length_squared() as f64).max(1e-5))
                            .min(angular)) as f32,
                        );
                        for j in i + 1..n {
                            state.update(j, head + qx(q, state.points[j] - head), false);
                            state.constrained(sk, j - 1, j - 1, j, false, limits, i);
                        }
                    }
                }
            }
        }
        state.cache(sk, chain, angular);
        if state.points[n - 1].distance_squared(target) as f64
            <= settings.min_distance * settings.min_distance
        {
            break;
        }
    }
    for i in 0..n - 1 {
        let q = state.joints[i].local;
        if q.is_finite() {
            sk.local[chain.as_slice()[i] as usize].rotation = q.normalize();
        }
    }
}
