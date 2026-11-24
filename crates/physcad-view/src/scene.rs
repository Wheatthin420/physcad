//! The sample scene both render paths draw: box, cylinder, torus.
//! Y-up world, primitives resting on the ground plane y=0.

use glam::Vec3;
use physcad::primitives::{Mesh, box_mesh, cylinder_mesh, torus_mesh};

#[derive(Clone, Debug)]
pub struct SceneItem {
    pub name: &'static str,
    pub mesh: Mesh,
    pub translation: Vec3,
    pub color: [f32; 3],
}

impl SceneItem {
    /// Model matrix: pure translation for now (no rotation/scale yet).
    pub fn model_matrix(&self) -> glam::Mat4 {
        glam::Mat4::from_translation(self.translation)
    }
}

pub fn sample_scene() -> Vec<SceneItem> {
    vec![
        SceneItem {
            name: "box",
            mesh: box_mesh(2.0, 2.0, 2.0),
            translation: Vec3::new(-3.4, 1.0, 0.0),
            color: [0.20, 0.45, 0.85],
        },
        SceneItem {
            name: "cylinder",
            mesh: cylinder_mesh(1.0, 2.5, 48),
            translation: Vec3::new(0.0, 1.25, 0.0),
            color: [0.85, 0.35, 0.25],
        },
        SceneItem {
            name: "torus",
            mesh: torus_mesh(1.0, 0.4, 64, 32),
            translation: Vec3::new(3.4, 0.4, 0.0),
            color: [0.25, 0.70, 0.35],
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_scene_primitives_are_closed_manifolds() {
        for item in sample_scene() {
            item.mesh
                .closed_manifold_check()
                .unwrap_or_else(|e| panic!("{}: {e}", item.name));
        }
    }
}
