//! Components for engine-independent skeleton modifier algorithms.
use crate::*;
use bevy_ecs::prelude::*;
use bevy_transform::components::Transform;
use glam::Vec3;
/// Settings shared by every modifier entity. Lower order values run first.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct ModifierCommon {
    /// Whether the modifier runs.
    pub active: bool,
    /// Pose blend weight, clamped to zero through one.
    pub influence: f32,
    /// Lower values run earlier in the modifier stack.
    pub order: i32,
}
impl Default for ModifierCommon {
    fn default() -> Self {
        Self {
            active: true,
            influence: 1.,
            order: 0,
        }
    }
}
/// Optional explicit rest transform. Otherwise the first observed local is used.
#[derive(Component, Clone, Copy, Debug)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct BoneRest(pub Transform);
/// World-space spring collider geometry.
#[derive(Component, Clone, Copy, Debug)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct SpringBoneCollider(pub SpringCollider);
/// Spring chain on plain Transform entities; state is allocated in the component.
#[derive(Component)]
#[require(ModifierCommon)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct SpringBoneChain {
    /// First joint entity in the chain.
    pub root: Entity,
    /// Last joint entity in the chain.
    pub end: Entity,
    /// Algorithm controls, with defaults matching Godot.
    pub settings: SpringBoneSettings,
    /// Collider entities; at most 32 per chain.
    pub colliders: Vec<Entity>,
    /// Additional force in world directions, in skeleton units.
    pub external_force: Vec3,
    /// Persistent fixed-capacity algorithm state; reset after changing a chain.
    pub state: SpringBoneState,
    pub(crate) binding: Option<super::chain::Binding>,
    pub(crate) frozen: [BonePose; 32],
    pub(crate) has_output: bool,
}
impl SpringBoneChain {
    /// Construct a chain with Godot defaults.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn new(root: Entity, end: Entity) -> Self {
        Self {
            root,
            end,
            settings: SpringBoneSettings::default(),
            colliders: Vec::new(),
            external_force: Vec3::ZERO,
            state: SpringBoneState::default(),
            binding: None,
            frozen: [BonePose::default(); 32],
            has_output: false,
        }
    }
}
/// Two-bone IK. The target and pole are ordinary world-space Transform entities.
#[derive(Component)]
#[require(ModifierCommon)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct TwoBoneIk {
    /// First joint entity in the chain.
    pub root: Entity,
    /// Middle joint entity of the two-bone solver.
    pub middle: Entity,
    /// Last joint entity in the chain.
    pub end: Entity,
    /// Transform entity whose world position supplies the goal.
    pub target: Entity,
    /// Optional Transform entity defining the bend plane.
    pub pole: Option<Entity>,
    /// Algorithm controls, with defaults matching Godot.
    pub settings: TwoBoneSettings,
    pub(crate) binding: Option<super::chain::Binding>,
}
impl TwoBoneIk {
    /// Construct an IK modifier.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn new(root: Entity, middle: Entity, end: Entity, target: Entity) -> Self {
        Self {
            root,
            middle,
            end,
            target,
            pole: None,
            settings: TwoBoneSettings::default(),
            binding: None,
        }
    }
}
/// Iterative chain IK with persistent solver state.
#[derive(Component)]
#[require(ModifierCommon)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct ChainIk {
    /// First joint entity in the chain.
    pub root: Entity,
    /// Last joint entity in the chain.
    pub end: Entity,
    /// Iterative algorithm to run.
    pub solver: IkSolver,
    /// Algorithm controls, with defaults matching Godot.
    pub settings: IterateSettings,
    /// Optional limits in root-to-end order.
    pub limits: Vec<Option<JointLimit>>,
    /// Transform entity whose world position supplies the goal.
    pub target: Entity,
    /// Persistent fixed-capacity algorithm state; reset after changing a chain.
    pub state: IterateState,
    pub(crate) binding: Option<super::chain::Binding>,
}
impl ChainIk {
    /// Construct a modifier with Godot's iteration defaults.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn new(root: Entity, end: Entity, target: Entity, solver: IkSolver) -> Self {
        Self {
            root,
            end,
            solver,
            settings: IterateSettings::default(),
            limits: Vec::new(),
            target,
            state: IterateState::default(),
            binding: None,
        }
    }
}
/// Orient a chain along a path placed by another Transform entity.
#[derive(Component)]
#[require(ModifierCommon)]
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
pub struct SplineIk {
    /// First joint entity in the chain.
    pub root: Entity,
    /// Last joint entity in the chain.
    pub end: Entity,
    /// Baked path data, allocated at setup.
    pub path: BakedPath,
    /// Transform entity placing the path in world space.
    pub path_entity: Entity,
    /// Algorithm controls, with defaults matching Godot.
    pub settings: SplineSettings,
    pub(crate) binding: Option<super::chain::Binding>,
}
impl SplineIk {
    /// Construct a spline modifier.
    /// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html).
    pub fn new(root: Entity, end: Entity, path: BakedPath, path_entity: Entity) -> Self {
        Self {
            root,
            end,
            path,
            path_entity,
            settings: SplineSettings::default(),
            binding: None,
        }
    }
}
