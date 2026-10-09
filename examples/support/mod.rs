//! Asset-free native examples. No imported rig, skin, texture, or animation clip.
use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};
use bevy_skeleton_modifiers::*;
#[derive(Resource)]
pub struct Demo {
    mode: u8,
    elapsed: f32,
    captured: bool,
    limits: bool,
}
#[derive(Component)]
struct RigSpace;
#[derive(Component)]
struct Goal;
#[derive(Component)]
struct PathSpace;
#[derive(Component)]
struct BoneMarker(Color);
#[derive(Component)]
struct ColliderMarker;
pub fn run(mode: u8, title: &str) {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.015, 0.02, 0.03)))
        .insert_resource(Demo {
            mode,
            elapsed: 0.,
            captured: false,
            limits: false,
        })
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: title.into(),
                    resolution: (1100, 800).into(),
                    ..default()
                }),
                ..default()
            }),
            SkeletonModifiersPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, animate)
        .add_systems(
            PostUpdate,
            draw.after(bevy_transform::TransformSystems::Propagate),
        )
        .run();
}
fn spawn_chain(
    commands: &mut Commands,
    space: Entity,
    count: usize,
    color: Color,
    mesh: &Handle<Mesh>,
    material: &Handle<StandardMaterial>,
) -> Vec<Entity> {
    let mut chain = Vec::new();
    let mut parent = space;
    for i in 0..count {
        let local = if i == 0 {
            Transform::IDENTITY
        } else {
            Transform::from_xyz(0., 0.5, 0.)
        };
        let bone = commands
            .spawn((
                local,
                Visibility::default(),
                BoneRest(local),
                ChildOf(parent),
                BoneMarker(color),
            ))
            .id();
        if i + 1 < count {
            commands.spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material.clone()),
                Transform::from_xyz(0., 0.25, 0.),
                ChildOf(bone),
            ));
        }
        chain.push(bone);
        parent = bone;
    }
    chain
}
fn setup(
    mut commands: Commands,
    demo: Res<Demo>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Transform::from_xyz(0., 3.8, 9.).looking_at(Vec3::new(0., 1.2, 0.), Vec3::Y),
    ));
    let mesh = meshes.add(Capsule3d::new(0.045, 0.41));
    for rig in 0..if demo.mode == 2 { 3 } else { 1 } {
        let x = if demo.mode == 2 {
            (rig as f32 - 1.) * 2.4
        } else {
            0.
        };
        let space = commands
            .spawn((
                Transform::from_xyz(x, 0., 0.),
                Visibility::default(),
                RigSpace,
            ))
            .id();
        let color = match rig {
            0 => Color::srgb(0.25, 0.85, 1.),
            1 => Color::srgb(1., 0.65, 0.3),
            _ => Color::srgb(0.7, 0.5, 1.),
        };
        let material = materials.add(StandardMaterial {
            base_color: color,
            unlit: true,
            ..default()
        });
        let count = if demo.mode == 1 { 3 } else { 6 };
        let chain = spawn_chain(&mut commands, space, count, color, &mesh, &material);
        let root = chain[0];
        let end = *chain.last().unwrap();
        match demo.mode {
            0 => {
                let collider = commands
                    .spawn((
                        Transform::from_xyz(x + 0.15, 1.3, 0.),
                        SpringBoneCollider(SpringCollider::Sphere {
                            radius: 0.25,
                            inside: false,
                        }),
                        ColliderMarker,
                    ))
                    .id();
                let mut spring = SpringBoneChain::new(root, end);
                spring.settings.stiffness = 1.5;
                spring.settings.drag = 0.15;
                spring.colliders.push(collider);
                commands.spawn(spring);
                let body_mesh = meshes.add(Capsule3d::new(0.16, 0.5));
                commands.spawn((
                    Mesh3d(body_mesh),
                    MeshMaterial3d(material.clone()),
                    Transform::from_xyz(-0.45, 0.3, 0.),
                    ChildOf(space),
                ));
            }
            1 => {
                let target = commands
                    .spawn((Transform::from_xyz(0.6, 0.6, 0.), Goal))
                    .id();
                let pole = commands.spawn(Transform::from_xyz(0., 0.5, 1.5)).id();
                let mut ik = TwoBoneIk::new(root, chain[1], end, target);
                ik.pole = Some(pole);
                commands.spawn(ik);
            }
            2 => {
                let target = commands
                    .spawn((Transform::from_xyz(x + 0.65, 1.8, 0.3), Goal))
                    .id();
                let solver = [IkSolver::Fabrik, IkSolver::Ccd, IkSolver::Jacobian][rig];
                let ik = ChainIk::new(root, end, target, solver);
                commands.spawn(ik);
            }
            _ => {
                let mut points = Vec::new();
                let mut tilts = Vec::new();
                for i in 0..160 {
                    let t = i as f32 / 159.;
                    let angle = t * 5.;
                    points.push(Vec3::new(
                        angle.sin() * 0.45,
                        t * 2.8,
                        (angle.cos() - 1.) * 0.45,
                    ));
                    tilts.push(t * 0.8);
                }
                let path = commands.spawn((Transform::IDENTITY, PathSpace)).id();
                commands.spawn(SplineIk::new(
                    root,
                    end,
                    BakedPath {
                        points,
                        tilts,
                        interval: 0.02,
                    },
                    path,
                ));
            }
        }
    }
}
type RigFilter = (With<RigSpace>, Without<Goal>, Without<PathSpace>);
type GoalFilter = (With<Goal>, Without<RigSpace>, Without<PathSpace>);
type PathFilter = (With<PathSpace>, Without<Goal>, Without<RigSpace>);
fn animate(
    mut commands: Commands,
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut demo: ResMut<Demo>,
    mut spaces: Query<&mut Transform, RigFilter>,
    mut goals: Query<&mut Transform, GoalFilter>,
    mut paths: Query<&mut Transform, PathFilter>,
    mut chains: Query<&mut ChainIk>,
    camera: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    mut exit: MessageWriter<AppExit>,
) {
    demo.elapsed += time.delta_secs();
    let t = demo.elapsed;
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
    if demo.mode == 0 {
        for mut p in &mut spaces {
            p.translation.x = t.sin() * 0.2;
            p.rotation = Quat::from_rotation_z((t * 0.7).sin() * 0.2);
        }
    }
    if demo.mode == 1
        && let Ok(window) = windows.single()
        && let Some(cursor) = window.cursor_position()
        && let Ok((camera, pose)) = camera.single()
        && let Ok(ray) = camera.viewport_to_world(pose, cursor)
        && ray.direction.z.abs() > 1e-6
    {
        let distance = -ray.origin.z / ray.direction.z;
        if distance > 0. {
            for mut target in &mut goals {
                target.translation = ray.get_point(distance);
            }
        }
    }
    if demo.mode == 2 {
        for (i, mut p) in goals.iter_mut().enumerate() {
            p.translation = Vec3::new(
                (i as f32 - 1.) * 2.4 + 0.55 * (t * 0.8).sin(),
                1.7 + 0.25 * t.cos(),
                0.4 * (t * 0.7).cos(),
            );
        }
        if keys.just_pressed(KeyCode::Space) {
            demo.limits = !demo.limits;
            for mut chain in &mut chains {
                chain.limits = if demo.limits {
                    vec![
                        Some(JointLimit::Cone {
                            angle: 0.9,
                            right_axis: SecondaryDirection::None,
                            right_axis_vector: Vec3::X,
                            rotation_offset: Quat::IDENTITY
                        });
                        6
                    ]
                } else {
                    Vec::new()
                };
                chain.state.reset();
            }
        }
    }
    for mut p in &mut paths {
        p.rotation = Quat::from_rotation_y(t.sin() * 0.15);
    }
    if t > 3.
        && !demo.captured
        && let Ok(path) = std::env::var("SKELETON_SCREENSHOT")
    {
        demo.captured = true;
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path))
            .observe(
                |_: On<bevy::render::view::screenshot::ScreenshotCaptured>,
                 mut exit: MessageWriter<AppExit>| {
                    exit.write(AppExit::Success);
                },
            );
    }
    if t > 15. && std::env::var_os("SKELETON_SCREENSHOT").is_some() {
        exit.write(AppExit::Success);
    }
}
fn draw(
    mut gizmos: Gizmos,
    bones: Query<(&GlobalTransform, &ChildOf, &BoneMarker)>,
    poses: Query<&GlobalTransform>,
    goals: Query<&GlobalTransform, With<Goal>>,
    colliders: Query<&GlobalTransform, With<ColliderMarker>>,
    splines: Query<&SplineIk>,
) {
    for (p, parent, color) in &bones {
        if let Ok(start) = poses.get(parent.parent()) {
            gizmos.line(start.translation(), p.translation(), color.0);
        }
    }
    for p in &goals {
        gizmos.sphere(
            Isometry3d::from_translation(p.translation()),
            0.09,
            Color::srgb(1., 0.85, 0.3),
        );
    }
    for p in &colliders {
        gizmos.sphere(
            Isometry3d::from_translation(p.translation()),
            0.25,
            Color::srgb(1., 0.4, 0.45),
        );
    }
    for spline in &splines {
        if let Ok(space) = poses.get(spline.path_entity) {
            for pair in spline.path.points.windows(2) {
                gizmos.line(
                    space.transform_point(pair[0]),
                    space.transform_point(pair[1]),
                    Color::srgb(0.5, 0.6, 0.65),
                );
            }
        }
    }
    gizmos.line(
        Vec3::new(-4., 0., 0.),
        Vec3::new(4., 0., 0.),
        Color::srgb(0.18, 0.22, 0.28),
    );
}
