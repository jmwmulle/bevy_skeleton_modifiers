# bevy_skeleton_modifiers

**Unreleased:** implementation and macOS mechanical checks pass, but Linux CCD warm-start reference replay exceeds the required tolerance. Publication is held pending that fix; see PORTING.md.

Spring-bone chains, two-bone IK, FABRIK, CCD, Jacobian IK and open-path spline IK, translated from Godot's MIT-licensed skeleton modifiers. The math core uses borrowed poses and glam. The default Bevy adapter works with any `ChildOf`/`Transform` hierarchy, including rigid parts without a skin.

## Bevy use

```rust,no_run
use bevy::prelude::*;
use bevy_skeleton_modifiers::*;

fn install(app: &mut App) {
    app.add_plugins(SkeletonModifiersPlugin);
}

fn attach(mut commands: Commands, root: Entity, middle: Entity,
          end: Entity, target: Entity) {
    commands.spawn(TwoBoneIk::new(root, middle, end, target));
}
```

The target and optional pole are ordinary world-space transform entities. Place a `SpringBoneChain`, `TwoBoneIk`, `ChainIk` or `SplineIk` on its own owner entity; its bone entities can belong to an existing rig. Use one modifier type per owner. Each requires `ModifierCommon`, automatically inserted with `active = true`, `influence = 1` and `order = 0`. Lower orders run first; entity identity breaks ties. Overlapping modifiers blend in that order.

Updates run in `PostUpdate`, after `AnimationSystems` and before transform propagation. Root, target and collider positions are computed from current local transforms, so movement affects the same frame. The adapter keeps the unmodified input separate from its last output; unchanged transforms do not accumulate their own modifications. If an application writes a new pose, do so before `SkeletonModifierSystems`. Writing exactly the previous output is indistinguishable from leaving it unchanged.

Add `BoneRest(Transform)` for explicit rest poses. Otherwise the first observed local pose becomes the rest pose; install before animation starts when using this fallback. `resolve_bone_by_name` finds a named descendant in an existing hierarchy. Replacing/despawning/reparenting a bound bone disables that modifier with one warning; create a new modifier for the replacement chain.

The default spring clock reads `Time<Virtual>`. Insert `ModifierClock::Manual` and write `ModifierDelta(dt)` before the modifier set to inject your own clock. Nonpositive or nonfinite delta freezes spring state and its last solved pose. IK responds to targets independently of time. `SpringBoneCollider` supplies spheres, capsules and planes; each chain names its collider entities.

| Bevy | bevy_skeleton_modifiers |
|---|---|
| 0.19 | 0.1 |

`--no-default-features` removes Bevy; use `SkeletonView`, `BonePose` and the `process_*` functions directly. Enable `gizmos` for `draw_bone_gizmos`, and add that system after transform propagation with a Bevy gizmo renderer installed.

## Bounds and conventions

- Core targets and poles are in skeleton coordinates; the Bevy adapter converts world positions. Quaternions use xyzw, distances use skeleton units, angles use radians and delta uses seconds.
- Chains and per-chain collider lists have a capacity of 32. Setup and path baking may allocate; core solves and stable Bevy updates reuse their storage. Reset persistent state after changing a core chain or simulation reference frame. `SpringCenter::Bone` uses a core skeleton index, or a root-to-end chain index in the Bevy adapter.
- Positive uniform root scale is supported. Nonuniform scale, shear and reflected skeleton coordinate systems are unsupported. Keep bone-local scale at one for spring/iterative length constraints.
- Spline IK supports open baked paths. Point/tilt arrays must have equal length. `BakedPath::from_curve` resamples finite-domain Bevy curves at approximately uniform arc length; the final segment can be shorter. Godot's pinned open-path sample-selection quirk is preserved, described in PORTING.md.
- Iterative solvers use angular limits and a bounded number of passes, so reaching a target can take multiple updates. Cone angle means the full opening angle. Terminal joints retain their input orientation unless spring/two-bone end extension is enabled.

## Examples

All examples build their meshes and rigs in code and require no downloaded assets.

- `cargo run --example spring_tail`: a moving capsule with a spring tail and sphere collider.
- `cargo run --example reaching_arm`: a two-bone limb reaches the pointer projected onto the XY plane.
- `cargo run --example chain_solvers`: FABRIK, CCD and Jacobian chains follow moving goals. Space toggles cone limits.
- `cargo run --example spline_tentacle`: a six-joint chain follows a helix with baked tilt.

Escape closes each example. Multisample anti-aliasing is disabled for reliable captures on the tested macOS/Metal host.

## Verification

Tests replay all 300 frames of 23 scenarios captured from the native Godot 4.7.2 modifier pipeline, plus shortest-arc quaternion references. They cover colliders, hinges, virtual ends, limits, warm starts and spline tilt. Tests also check fixed-target stability, degenerate geometry, scale, input/output separation, pauses, ordering and plain Bevy hierarchies. See PORTING.md for numerical tolerances and intentional differences. The reference trajectories were not changed to fit the port.

Rust 1.98.1 is used locally; CI checks Linux and the declared Rust 1.95 minimum. Numerical comparisons verify the recorded scenarios, not every possible rig or visual result.

One rough core-only timing on the development Apple Silicon host: 75 six-joint chains (spring, FABRIK, CCD and Jacobian) over 1,000 updates took 331.208 ms total, approximately 0.3312 ms per batch. This uses `rustc -O` and `tools/timing.rs`, excluding setup, Bevy and rendering; it is not a frame-rate promise.

## Origins and credits

Upstream: [Godot Engine 4.7.2-stable](https://github.com/godotengine/godot/tree/ed1daf0bf001b61586d9930840f2f1394092c079), commit `ed1daf0bf001b61586d9930840f2f1394092c079`.

Original algorithm authorship and copyright remain with the Godot Engine contributors, Juan Linietsky and Ariel Manzur. The relevant file histories credit Silc Lizard (Tokage) Renew, Thaddeus Crews, Rémi Verschelde, LuoZhihao, Lyuma, kobewi, Lukas Tenbrink and Michael Alexsander. The full pinned Godot contributor list is preserved in AUTHORS.upstream.md; file-specific history evidence is retained in Git under `tools/goldens/upstream/file-authors.json`.

Godot documentation: [SkeletonModifier3D](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html), [SpringBoneSimulator3D](https://docs.godotengine.org/en/stable/classes/class_springbonesimulator3d.html), [TwoBoneIK3D](https://docs.godotengine.org/en/stable/classes/class_twoboneik3d.html), [IterateIK3D](https://docs.godotengine.org/en/stable/classes/class_iterateik3d.html), [SplineIK3D](https://docs.godotengine.org/en/stable/classes/class_splineik3d.html). Godot editor features and its scene model are outside this port. Not affiliated with or endorsed by the Godot Foundation or Godot Engine project.

Design and implementation plan: Claude (Anthropic). Rust port, Bevy integration, reference harnesses, implementation and verification: Codex (OpenAI). The human initiator proposed having AI tools examine and port existing animation libraries and authorized release; they did not perform the porting. These credits describe the work performed and do not designate AI systems as copyright holders.

## Licence

The combined distribution is MIT with Godot's copyright and permission notices retained. Original additions are dedicated under CC0 1.0 Universal, to the extent that the publisher can waive rights in them. Commercial and noncommercial use, modification and redistribution are permitted. No credit to the human initiator or AI tools is required. See LICENSING.md for scope and LICENSE-CC0 for the waiver and fallback. This dedication preserves upstream and third-party rights and the upstream notice requirement.
