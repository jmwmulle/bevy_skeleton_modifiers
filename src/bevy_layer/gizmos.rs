//! Optional debug drawing; the application installs Bevy's gizmo renderer.
use super::BoneRest;
use bevy_color::Color;
use bevy_ecs::prelude::*;
use bevy_gizmos::gizmos::Gizmos;
use bevy_transform::components::GlobalTransform;
/// Draw lines to parents for entities tagged with `BoneRest`.
/// Add this system after transform propagation when using Bevy's gizmo renderer.
/// See the [Godot class reference](https://docs.godotengine.org/en/stable/classes/class_skeleton3d.html).
pub fn draw_bone_gizmos(
    mut gizmos: Gizmos,
    bones: Query<(&GlobalTransform, &ChildOf), With<BoneRest>>,
    parents: Query<&GlobalTransform>,
) {
    for (pose, parent) in &bones {
        if let Ok(p) = parents.get(parent.parent()) {
            gizmos.line(
                p.translation(),
                pose.translation(),
                Color::srgb(0.25, 0.75, 1.),
            );
        }
    }
}
