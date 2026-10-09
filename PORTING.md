# Porting record

## Source and structure

Every upstream-derived Rust file names its source and preserves the Godot notice. All sources are pinned to `godotengine/godot` commit `ed1daf0bf001b61586d9930840f2f1394092c079` (4.7.2-stable).

| Rust module | Godot source |
|---|---|
| core/skeleton.rs, axes.rs | skeleton_modifier_3d, skeleton_3d, ik_modifier_3d, spring_bone_simulator_3d |
| core/godot_math.rs | core/math/quaternion.cpp, quaternion.h, basis.cpp |
| core/spring_bone.rs | spring_bone_simulator_3d |
| core/collision.rs | spring_bone_collision_sphere_3d, spring_bone_collision_capsule_3d, spring_bone_collision_plane_3d |
| core/two_bone.rs | two_bone_ik_3d and mutable-rest helpers |
| core/iterate.rs | chain_ik_3d, iterate_ik_3d, fabr_ik_3d, ccd_ik_3d, jacobian_ik_3d |
| core/limits.rs | joint_limitation_3d, joint_limitation_cone_3d |
| core/spline.rs | spline_ik_3d; Bevy curve baking is an original addition |
| bevy_layer/ | Original adapter replacing Godot's scene/modifier pipeline |

Borrowed TRS arrays and parent indices replace Skeleton3D. Fixed arrays replace per-chain dynamic storage, with a 32-joint capacity. Source snapshots are retained verbatim in Git with SHA-256 provenance; snapshots are excluded from the crate. Distribution notices and the full pinned Godot author list remain included.

Godot Basis stores rows. Explicit scalar vector, quaternion and affine operations reproduce its evaluation order, including division during vector normalization, double intermediates where upstream uses `double`, shortest-arc thresholds, swing extraction, slerp's double `1.0 - weight`, Gram–Schmidt orthonormalization and matrix-to-quaternion conversion. No matrix-memory reinterpretation is used. A `FromParent` extension uses the rest basis transpose, matching Godot's `Basis::xform_inv`, before normalization. Godot's macOS release disables fused floating-point contraction; no FMA-based explanation or correction is applied.

## Reference verification

The verified official Godot 4.7.2 executable generated 23 asset-free scenarios, each with 300 native modifier outputs at fixed 60 fps. The actual delta, input locals, output world positions/quaternions, target/pole, root placement and spline baked data are recorded. Setup registers the native modifier before recording and captures its signal before Skeleton3D restores input. A second complete generation produced identical hashes. Fixtures are losslessly compressed JSON totaling 2,398,643 bytes, plus small math references.

Spring, two-bone and spline comparisons permit position error of `bone_length × 1e-3` and quaternion angle error of `1e-3` radians. Iterative cases use `1e-3` absolute position and `1e-3` radians, except `ccd_limits`, which uses `1e-2` for both. Worst observed macOS error in that case was 0.005315292 position units and 0.005895572 radians. Limits and repeated shortest-arc updates amplify tiny rounding differences near branch boundaries; the wider tolerance is explicit rather than altering the reference motion. On macOS, other iterative cases measured below `4.8e-5` position and `8.0e-5` radians.

**Unresolved release blocker:** Linux x86_64 replay of the macOS reference diverges in CCD warm-start motion. The original `1e-3` angular bound fails at frame 82. A diagnostic run with `1e-2` also failed at frame 118, bone 3: position error 0.001499805, angular error 0.013929696 radians. That widened bound was rejected and restored to `1e-3`; reference fixtures remain unchanged. Accumulated platform floating-point differences are suspected, but the exact cause is not established. Linux Bevy integration, fixed-target stability, formatting, lint and Rust 1.95 checks passed. Full Linux validation and publication are held pending resolution. Independent fixed-target stability tests retain the `1e-5` drift bound. No further visual testing or expanded investigation is authorized within the current testing budget.

Additional tests cover quaternion opposite vectors, influence endpoints, invalid chains/cycles, zero-length segments, missing/collinear poles, reachable/unreachable fixed-target stability, positive uniform root scales 0.01/1/2, Bevy same-frame root movement, order, influence, input/output separation, frozen springs and binding invalidation. These checks do not establish artistic quality or universal behavior on arbitrary rigs.

## Intentional differences and bounds

- Nonpositive or nonfinite spring delta freezes pose and state. The Bevy adapter retains the last solved spring pose during a pause and blends its influence once. Godot's editor/pipeline callback behavior is not reproduced.
- Spring simulation coordinates normalize positive uniform skeleton-root scale, preserving response in bone units. Nonuniform scale, shear and reflected coordinate systems are unsupported. Local scale should remain one for spring and iterative length constraints.
- A free iterative chain with a target outside total reach uses its straight closest-reachable configuration, followed by the usual angular-change clamp. This prevents an unreachable-target oscillation observed in the pinned Jacobian behavior; constrained targets retain the upstream iterative path. Original reference targets do not exercise this changed branch.
- Missing or collinear two-bone poles use the input bend direction, then a deterministic perpendicular. Degenerate segments return finite unchanged output. Invalid geometry/index chains are rejected. Invalid capsule height/radius is clamped to a finite shape; an inside sphere smaller than the tail radius clamps its available radius to zero.
- Bevy detects removal/reparenting or changed endpoints and disables the modifier with a warning, rather than reading a stale binding. Missing targets/colliders/path entities and multiple modifier types on one owner also disable it. Use distinct owner entities for ordered operations. Duplicate names resolve by entity order.
- Bevy cannot distinguish an application deliberately rewriting exactly the previous output from an untouched transform. Other rewritten local poses become the new unmodified input. Rest poses are explicit BoneRest values or captured on first observation.
- World/node/bone spring centers are supported. Reset persistent state when changing a core chain, center or reference frame. External force and gravity directions use world axes; values are expressed in skeleton units.
- Spline processing uses open paths only. Godot 4.7.2's open-path branch assigns `nearest_next = nearest`, rather than `nearest + 1`, so its forward sampling does not interpolate to the next baked point. This behavior is preserved and can produce stepped direction changes on sparse paths. The test reuses Godot's baked points/tilts, while Bevy curve baking is an independent setup routine tested for arc-length spacing.
- Spring and two-bone solvers expose terminal extension. The requested minimal iterative and spline interfaces do not expose Godot's terminal-extension properties. The terminal bone keeps its input orientation.
- Setting/overrides and path vectors allocate at setup; stable update paths reuse fixed storage and the adapter's pose cache. Dynamically adding modifier owners/bones can grow that cache during setup of those owners.

## Exclusions

No Godot editor, inspector, binding, compatibility includes, property validation, scene import or skin import is ported. Godot curves are supplied as baked points or baked from Bevy curves. Legacy SkeletonIK3D, LookAt/Aim/CopyTransform/ConvertTransform/BoneConstraint/LimitAngularVelocity/Retarget, modifier target editor helpers, closed splines and other joint-limitation resources are outside this release. No upstream article/documentation prose or illustrations are copied. No issues or messages have been sent upstream.
