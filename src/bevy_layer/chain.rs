//! Hierarchy access and fixed-capacity bone bindings.
use crate::ChainError;
use bevy_ecs::prelude::*;
use bevy_transform::components::Transform;
use glam::Affine3A;
#[derive(Clone, Copy)]
pub(crate) struct Binding {
    pub bones: [Entity; 32],
    pub parents: [Option<Entity>; 32],
    pub len: usize,
}
impl Binding {
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn new(world: &World, root: Entity, end: Entity) -> Result<Self, ChainError> {
        let mut result = Self {
            bones: [Entity::PLACEHOLDER; 32],
            parents: [None; 32],
            len: 0,
        };
        let mut e = end;
        loop {
            if world.get::<Transform>(e).is_none() {
                return Err(ChainError::InvalidIndex);
            }
            if result.bones[..result.len].contains(&e) {
                return Err(ChainError::Cycle);
            }
            if result.len == 32 {
                return Err(ChainError::TooLong);
            }
            result.bones[result.len] = e;
            result.len += 1;
            if e == root {
                break;
            }
            e = world
                .get::<ChildOf>(e)
                .map(ChildOf::parent)
                .ok_or(ChainError::NotDescendant)?;
        }
        result.bones[..result.len].reverse();
        for i in 0..result.len {
            result.parents[i] = world.get::<ChildOf>(result.bones[i]).map(ChildOf::parent)
        }
        Ok(result)
    }
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
    pub fn valid(self, world: &World, root: Entity, end: Entity) -> bool {
        self.bones[0] == root
            && self.bones[self.len - 1] == end
            && (0..self.len).all(|i| {
                world.get::<Transform>(self.bones[i]).is_some()
                    && world.get::<ChildOf>(self.bones[i]).map(ChildOf::parent) == self.parents[i]
            })
    }
}
pub(crate) fn current_global(world: &World, entity: Entity) -> Option<Affine3A> {
    let mut e = entity;
    let mut transform = Affine3A::IDENTITY;
    for _ in 0..1024 {
        let local = world.get::<Transform>(e)?;
        transform = local.compute_affine() * transform;
        let Some(parent) = world.get::<ChildOf>(e) else {
            return transform.is_finite().then_some(transform);
        };
        e = parent.parent();
    }
    None
}
/// Find a named descendant in an existing hierarchy, without relying on skinning.
/// Duplicate names resolve to the first entity in deterministic entity order.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
pub fn resolve_bone_by_name(world: &World, root: Entity, name: &str) -> Option<Entity> {
    let mut selected = None;
    for e in world.iter_entities() {
        if e.get::<Name>().is_none_or(|n| n.as_str() != name) {
            continue;
        }
        let mut ancestor = Some(e.id());
        for _ in 0..1024 {
            let Some(a) = ancestor else { break };
            if a == root {
                if selected.is_none_or(|old: Entity| e.id().to_bits() < old.to_bits()) {
                    selected = Some(e.id())
                }
                break;
            }
            ancestor = world.get::<ChildOf>(a).map(ChildOf::parent);
        }
    }
    selected
}
