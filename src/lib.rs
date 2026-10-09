//! Godot-derived skeleton modifiers for plain hierarchies and Bevy rigs.
//! Upstream: <https://docs.godotengine.org/en/stable/classes/class_skeletonmodifier3d.html>.
#![forbid(unsafe_code)]
#![warn(missing_docs)]
pub mod core;
pub use core::*;

#[cfg(feature = "bevy")]
pub mod bevy_layer;
#[cfg(feature = "bevy")]
pub use bevy_layer::*;
