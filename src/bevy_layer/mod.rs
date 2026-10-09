//! Ordered Bevy modifier stack, after animation and before transform propagation.
mod chain;
mod components;
use crate::*;
use bevy_app::{AnimationSystems, App, Plugin, PostUpdate};
use bevy_ecs::{prelude::*, query::QueryState};
use bevy_time::{Time, Virtual};
use bevy_transform::{TransformSystems, components::Transform};
pub use chain::resolve_bone_by_name;
pub use components::*;
use glam::{Affine3A, Vec3};
use std::collections::HashMap;
/// Which clock drives spring simulation.
#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub enum ModifierClock {
    #[default]
    /// Read Bevy virtual time.
    Virtual,
    /// Read the application-supplied delta.
    Manual,
}
/// Seconds per update in Manual clock mode.
#[derive(Resource, Clone, Copy, Debug, Default)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct ModifierDelta(pub f32);
/// Modifier schedule set, between Bevy animation and transform propagation.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct SkeletonModifierSystems;
/// Installs the stack for any ChildOf/Transform hierarchy.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct SkeletonModifiersPlugin;
#[derive(Clone, Copy)]
struct CachedPose {
    input: Transform,
    output: Transform,
    rest: BonePose,
}
type ModifierFilter = Or<(
    With<SpringBoneChain>,
    With<TwoBoneIk>,
    With<ChainIk>,
    With<SplineIk>,
)>;
#[derive(Resource)]
struct Stack {
    query: QueryState<Entity, ModifierFilter>,
    entities: Vec<Entity>,
    poses: HashMap<Entity, CachedPose>,
}
impl FromWorld for Stack {
    fn from_world(world: &mut World) -> Self {
        Self {
            query: world.query_filtered(),
            entities: Vec::new(),
            poses: HashMap::new(),
        }
    }
}
impl Plugin for SkeletonModifiersPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ModifierClock>()
            .init_resource::<ModifierDelta>()
            .init_resource::<Stack>()
            .configure_sets(
                PostUpdate,
                SkeletonModifierSystems
                    .after(AnimationSystems)
                    .before(TransformSystems::Propagate),
            )
            .add_systems(PostUpdate, update.in_set(SkeletonModifierSystems));
    }
}
fn bone(t: Transform) -> BonePose {
    BonePose {
        translation: t.translation,
        rotation: t.rotation,
        scale: t.scale,
    }
}
fn transform(p: BonePose) -> Transform {
    Transform {
        translation: p.translation,
        rotation: p.rotation,
        scale: p.scale,
    }
}
fn disable(world: &mut World, owner: Entity, reason: &str) {
    if let Some(mut c) = world.get_mut::<ModifierCommon>(owner) {
        c.active = false;
    }
    bevy_log::warn!("Skeleton modifier {owner:?} disabled: {reason}");
}
fn update(world: &mut World) {
    let dt = match *world.resource::<ModifierClock>() {
        ModifierClock::Manual => world.resource::<ModifierDelta>().0,
        ModifierClock::Virtual => world
            .get_resource::<Time<Virtual>>()
            .map_or(0., Time::delta_secs),
    };
    world.resource_scope(|world, mut stack: Mut<Stack>| {
        stack.poses.retain(|&entity, cached| {
            let Some(mut current) = world.get_mut::<Transform>(entity) else {
                return false;
            };
            if *current != cached.output {
                cached.input = *current;
            }
            *current = cached.input;
            true
        });
        stack.entities.clear();
        let Stack {
            query, entities, ..
        } = &mut *stack;
        entities.extend(query.iter(world));
        entities.sort_unstable_by_key(|&e| {
            (
                world.get::<ModifierCommon>(e).map_or(0, |c| c.order),
                e.to_bits(),
            )
        });
        for index in 0..stack.entities.len() {
            let owner = stack.entities[index];
            let common = world
                .get::<ModifierCommon>(owner)
                .copied()
                .unwrap_or_default();
            if !common.active {
                continue;
            }
            let type_count = usize::from(world.get::<SpringBoneChain>(owner).is_some())
                + usize::from(world.get::<TwoBoneIk>(owner).is_some())
                + usize::from(world.get::<ChainIk>(owner).is_some())
                + usize::from(world.get::<SplineIk>(owner).is_some());
            if type_count != 1 {
                disable(world, owner, "use one modifier type per owner entity");
                continue;
            }
            // Distinct entities supply explicit stack order.
            let (root, end, cached) = if let Some(c) = world.get::<SpringBoneChain>(owner) {
                (c.root, c.end, c.binding)
            } else if let Some(c) = world.get::<TwoBoneIk>(owner) {
                (c.root, c.end, c.binding)
            } else if let Some(c) = world.get::<ChainIk>(owner) {
                (c.root, c.end, c.binding)
            } else if let Some(c) = world.get::<SplineIk>(owner) {
                (c.root, c.end, c.binding)
            } else {
                continue;
            };
            let binding = if let Some(b) = cached {
                if !b.valid(world, root, end) {
                    disable(
                        world,
                        owner,
                        "bone was despawned, reparented, or the binding changed",
                    );
                    continue;
                }
                b
            } else {
                match chain::Binding::new(world, root, end) {
                    Ok(b) => b,
                    Err(_) => {
                        disable(world, owner, "invalid chain or more than 32 bones");
                        continue;
                    }
                }
            };
            let n = binding.len;
            let Some(root_global) = binding.parents[0].map_or(Some(Affine3A::IDENTITY), |p| {
                chain::current_global(world, p)
            }) else {
                disable(world, owner, "missing ancestor transform");
                continue;
            };
            let mut parents = [None; 32];
            let mut rest = [BonePose::default(); 32];
            let mut local = [BonePose::default(); 32];
            for i in 0..n {
                let entity = binding.bones[i];
                let t = *world.get::<Transform>(entity).unwrap();
                let initial_rest = world.get::<BoneRest>(entity).map_or(t, |r| r.0);
                let cached = stack.poses.entry(entity).or_insert(CachedPose {
                    input: t,
                    output: t,
                    rest: bone(initial_rest),
                });
                parents[i] = if i == 0 { None } else { Some(i as u16 - 1) };
                rest[i] = cached.rest;
                local[i] = bone(t);
            }
            let input = local;
            let mut sk = SkeletonView {
                parents: &parents[..n],
                rest: &rest[..n],
                local: &mut local[..n],
                root_global,
            };
            if world.get::<SpringBoneChain>(owner).is_some() {
                let mut cols = [(SpringCollider::Plane, Affine3A::IDENTITY); 32];
                let mut count = 0;
                let c = world.get::<SpringBoneChain>(owner).unwrap();
                if c.colliders.len() > 32 {
                    disable(world, owner, "more than 32 colliders");
                    continue;
                }
                let mut valid = true;
                for &e in &c.colliders {
                    if let Some(col) = world.get::<SpringBoneCollider>(e)
                        && let Some(p) = chain::current_global(world, e)
                    {
                        cols[count] = (col.0, p);
                        count += 1;
                    } else {
                        valid = false;
                        break;
                    }
                }
                if !valid {
                    disable(world, owner, "missing collider transform");
                    continue;
                }
                let mut c = world.get_mut::<SpringBoneChain>(owner).unwrap();
                c.binding = Some(binding);
                if (!dt.is_finite() || dt <= 0.) && c.has_output {
                    sk.local.copy_from_slice(&c.frozen[..n]);
                } else {
                    let chain = sk.chain(0, n as u16 - 1).unwrap();
                    let SpringBoneChain {
                        settings,
                        state,
                        external_force,
                        ..
                    } = &mut *c;
                    process_spring_bones(
                        &mut sk,
                        &chain,
                        settings,
                        state,
                        &cols[..count],
                        *external_force,
                        dt,
                    );
                    c.frozen[..n].copy_from_slice(sk.local);
                    c.has_output = true;
                }
            } else if let Some(c) = world.get::<TwoBoneIk>(owner) {
                let Some(middle) = binding.bones[..n].iter().position(|&e| e == c.middle) else {
                    disable(world, owner, "middle bone is outside the chain");
                    continue;
                };
                let Some(target) = chain::current_global(world, c.target) else {
                    disable(world, owner, "missing target");
                    continue;
                };
                let pole = c.pole.and_then(|p| chain::current_global(world, p));
                if c.pole.is_some() && pole.is_none() {
                    disable(world, owner, "missing pole");
                    continue;
                }
                let target = root_global
                    .inverse()
                    .transform_point3(Vec3::from(target.translation));
                let pole = pole.map(|p| {
                    root_global
                        .inverse()
                        .transform_point3(Vec3::from(p.translation))
                });
                let mut c = world.get_mut::<TwoBoneIk>(owner).unwrap();
                c.binding = Some(binding);
                process_two_bone(
                    &mut sk,
                    0,
                    middle as u16,
                    n as u16 - 1,
                    target,
                    pole,
                    &c.settings,
                );
            } else if let Some(c) = world.get::<ChainIk>(owner) {
                let Some(target) = chain::current_global(world, c.target) else {
                    disable(world, owner, "missing target");
                    continue;
                };
                let target = root_global
                    .inverse()
                    .transform_point3(Vec3::from(target.translation));
                let mut c = world.get_mut::<ChainIk>(owner).unwrap();
                c.binding = Some(binding);
                let chain = sk.chain(0, n as u16 - 1).unwrap();
                let ChainIk {
                    solver,
                    settings,
                    limits,
                    state,
                    ..
                } = &mut *c;
                process_chain_ik(&mut sk, &chain, *solver, settings, limits, target, state);
            } else if let Some(c) = world.get::<SplineIk>(owner) {
                let Some(path_pose) = chain::current_global(world, c.path_entity) else {
                    disable(world, owner, "missing path transform");
                    continue;
                };
                let mut c = world.get_mut::<SplineIk>(owner).unwrap();
                c.binding = Some(binding);
                let chain = sk.chain(0, n as u16 - 1).unwrap();
                process_spline_ik(
                    &mut sk,
                    &chain,
                    &c.path,
                    root_global.inverse() * path_pose,
                    &c.settings,
                );
            }
            apply_influence(&input[..n], sk.local, common.influence);
            for (i, &pose) in local.iter().take(n).enumerate() {
                *world.get_mut::<Transform>(binding.bones[i]).unwrap() = transform(pose);
            }
        }
        for (&entity, cached) in &mut stack.poses {
            if let Some(t) = world.get::<Transform>(entity) {
                cached.output = *t;
            }
        }
    });
}

#[cfg(feature = "gizmos")]
mod gizmos;
#[cfg(feature = "gizmos")]
pub use gizmos::draw_bone_gizmos;
