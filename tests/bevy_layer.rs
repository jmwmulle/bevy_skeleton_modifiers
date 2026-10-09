#![cfg(feature = "bevy")]
use bevy_app::App;
use bevy_ecs::prelude::*;
use bevy_skeleton_modifiers::*;
use bevy_time::TimePlugin;
use bevy_transform::{
    TransformPlugin,
    components::{GlobalTransform, Transform},
};
use glam::{Quat, Vec3};
fn app() -> App {
    let mut app = App::new();
    app.add_plugins((TimePlugin, TransformPlugin, SkeletonModifiersPlugin));
    app.insert_resource(ModifierClock::Manual);
    app.insert_resource(ModifierDelta(1. / 60.));
    app
}
fn rig(app: &mut App) -> [Entity; 4] {
    let w = app.world_mut();
    let space = w
        .spawn((
            Transform::IDENTITY,
            GlobalTransform::default(),
            Name::new("rig"),
        ))
        .id();
    let a = w
        .spawn((
            Transform::IDENTITY,
            GlobalTransform::default(),
            ChildOf(space),
            Name::new("root"),
        ))
        .id();
    let b = w
        .spawn((
            Transform::from_translation(Vec3::Y * 0.5),
            GlobalTransform::default(),
            ChildOf(a),
            Name::new("middle"),
        ))
        .id();
    let c = w
        .spawn((
            Transform::from_translation(Vec3::Y * 0.5),
            GlobalTransform::default(),
            ChildOf(b),
            Name::new("end"),
        ))
        .id();
    [space, a, b, c]
}
fn target(app: &mut App, v: Vec3) -> Entity {
    app.world_mut()
        .spawn((Transform::from_translation(v), GlobalTransform::default()))
        .id()
}
#[test]
fn moving_root_has_no_frame_lag() {
    let mut app = app();
    let [space, a, b, c] = rig(&mut app);
    let t = target(&mut app, Vec3::new(0.5, 0.5, 0.));
    let mut ik = TwoBoneIk::new(a, b, c, t);
    ik.pole = Some(target(&mut app, Vec3::Z));
    app.world_mut().spawn(ik);
    app.update();
    app.world_mut()
        .get_mut::<Transform>(space)
        .unwrap()
        .translation = Vec3::X;
    app.world_mut().get_mut::<Transform>(t).unwrap().translation = Vec3::new(1.5, 0.5, 0.);
    app.update();
    let end = app.world().get::<GlobalTransform>(c).unwrap().translation();
    assert!(end.distance(Vec3::new(1.5, 0.5, 0.)) < 1e-4);
}
#[test]
fn modifiers_run_in_order_and_respect_influence() {
    let mut app = app();
    let [_, a, b, c] = rig(&mut app);
    let t1 = target(&mut app, Vec3::new(0.5, 0.5, 0.));
    let t2 = target(&mut app, Vec3::new(-0.5, 0.5, 0.));
    let p = target(&mut app, Vec3::Z);
    let mut first = TwoBoneIk::new(a, b, c, t1);
    first.pole = Some(p);
    let mut second = TwoBoneIk::new(a, b, c, t2);
    second.pole = Some(p);
    app.world_mut().spawn((
        first,
        ModifierCommon {
            order: 10,
            ..Default::default()
        },
    ));
    let last = app
        .world_mut()
        .spawn((
            second,
            ModifierCommon {
                order: 20,
                ..Default::default()
            },
        ))
        .id();
    app.update();
    assert!(
        app.world()
            .get::<GlobalTransform>(c)
            .unwrap()
            .translation()
            .distance(Vec3::new(-0.5, 0.5, 0.))
            < 1e-4
    );
    app.world_mut()
        .get_mut::<ModifierCommon>(last)
        .unwrap()
        .influence = 0.;
    app.update();
    assert!(
        app.world()
            .get::<GlobalTransform>(c)
            .unwrap()
            .translation()
            .distance(Vec3::new(0.5, 0.5, 0.))
            < 1e-4
    );
}
#[test]
fn does_not_accumulate_own_output() {
    let mut app = app();
    let [_, a, b, c] = rig(&mut app);
    let t = target(&mut app, Vec3::new(0.5, 0.5, 0.1));
    let mut ik = TwoBoneIk::new(a, b, c, t);
    ik.pole = Some(target(&mut app, Vec3::Z));
    app.world_mut().spawn((
        ik,
        ModifierCommon {
            influence: 0.5,
            ..Default::default()
        },
    ));
    let mut at_ten = None;
    for i in 0..300 {
        app.update();
        let output = [
            *app.world().get::<Transform>(a).unwrap(),
            *app.world().get::<Transform>(b).unwrap(),
        ];
        if i == 10 {
            at_ten = Some(output)
        }
        if i > 10 {
            assert_eq!(Some(output), at_ten)
        }
    }
}
#[test]
fn external_rewrite_becomes_new_input() {
    let mut app = app();
    let [_, a, b, c] = rig(&mut app);
    let t = target(&mut app, Vec3::new(0.5, 0.5, 0.));
    app.world_mut().spawn((
        TwoBoneIk::new(a, b, c, t),
        ModifierCommon {
            influence: 0.,
            ..Default::default()
        },
    ));
    app.update();
    let rotation = Quat::from_rotation_x(0.4);
    app.world_mut().get_mut::<Transform>(b).unwrap().rotation = rotation;
    app.update();
    assert_eq!(app.world().get::<Transform>(b).unwrap().rotation, rotation);
    app.update();
    assert_eq!(app.world().get::<Transform>(b).unwrap().rotation, rotation);
}
#[test]
fn manual_clock_zero_delta_freezes_spring_bones() {
    let mut app = app();
    let [_, a, _, c] = rig(&mut app);
    let mut spring = SpringBoneChain::new(a, c);
    spring.external_force = Vec3::X;
    app.world_mut().spawn(spring);
    for _ in 0..20 {
        app.update();
    }
    let pose = *app.world().get::<Transform>(a).unwrap();
    app.world_mut().resource_mut::<ModifierDelta>().0 = 0.;
    for _ in 0..100 {
        app.update();
        assert_eq!(*app.world().get::<Transform>(a).unwrap(), pose)
    }
}
#[test]
fn despawned_chain_member_disables_without_panic() {
    let mut app = app();
    let [_, a, b, c] = rig(&mut app);
    let owner = app.world_mut().spawn(SpringBoneChain::new(a, c)).id();
    app.update();
    app.world_mut().despawn(b);
    app.update();
    assert!(!app.world().get::<ModifierCommon>(owner).unwrap().active);
    app.update();
}
#[test]
fn works_on_rigid_part_hierarchy_without_skin() {
    let mut app = app();
    let [space, a, b, c] = rig(&mut app);
    let t = target(&mut app, Vec3::new(0.5, 0.5, 0.));
    let p = target(&mut app, Vec3::Z);
    let mut ik = TwoBoneIk::new(a, b, c, t);
    ik.pole = Some(p);
    app.world_mut().spawn(ik);
    app.update();
    assert!(
        app.world()
            .get::<GlobalTransform>(c)
            .unwrap()
            .translation()
            .distance(Vec3::new(0.5, 0.5, 0.))
            < 1e-4
    );
    assert_eq!(resolve_bone_by_name(app.world(), space, "middle"), Some(b));
}
#[test]
fn reparented_bone_disables_without_panic() {
    let mut app = app();
    let [space, a, b, c] = rig(&mut app);
    let owner = app.world_mut().spawn(SpringBoneChain::new(a, c)).id();
    app.update();
    app.world_mut().entity_mut(b).insert(ChildOf(space));
    app.update();
    assert!(!app.world().get::<ModifierCommon>(owner).unwrap().active);
}

#[test]
fn multiple_modifier_types_on_one_owner_are_disabled() {
    let mut app = app();
    let [_, a, b, c] = rig(&mut app);
    let t = target(&mut app, Vec3::new(0.5, 0.5, 0.));
    let owner = app
        .world_mut()
        .spawn((TwoBoneIk::new(a, b, c, t), SpringBoneChain::new(a, c)))
        .id();
    app.update();
    assert!(!app.world().get::<ModifierCommon>(owner).unwrap().active);
    assert_eq!(
        app.world().get::<Transform>(a).unwrap().rotation,
        Quat::IDENTITY
    );
}

#[test]
fn world_target_conversion_is_invariant_to_uniform_root_scale() {
    let mut rotations = Vec::new();
    for scale in [0.01, 1., 2.] {
        let mut app = app();
        let [space, a, b, c] = rig(&mut app);
        app.world_mut().get_mut::<Transform>(space).unwrap().scale = Vec3::splat(scale);
        let t = target(&mut app, Vec3::new(0.5, 0.5, 0.1) * scale);
        let p = target(&mut app, Vec3::Z * scale);
        let mut ik = TwoBoneIk::new(a, b, c, t);
        ik.pole = Some(p);
        app.world_mut().spawn(ik);
        app.update();
        let reached = app.world().get::<GlobalTransform>(c).unwrap().translation() / scale;
        assert!(reached.distance(Vec3::new(0.5, 0.5, 0.1)) < 1e-5);
        rotations.push(app.world().get::<Transform>(a).unwrap().rotation);
    }
    assert!(rotations[0].dot(rotations[1]).abs() > 1. - 1e-6);
    assert!(rotations[2].dot(rotations[1]).abs() > 1. - 1e-6);
}
