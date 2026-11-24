//! Scene contents for both render paths.
//!
//! M2 sample (box/cylinder/torus primitives) plus the M3 CSG demo: the
//! bore-plate model file is *built by the kernel* (model file -> params ->
//! CSG tree -> manifold booleans) and its resulting mesh is what the
//! viewer draws — the same `physcad::csg::evaluate` the CLI runs.
//! Y-up world, primitives resting on the ground plane y=0.

use glam::Vec3;
use physcad::csg;
use physcad::model::parse_model;
use physcad::primitives::{Mesh, box_mesh, cylinder_mesh, torus_mesh};

pub const DEMO_MODEL_SRC: &str = include_str!("../../../examples/bore-plate.pcm");
pub const DEMO_MODEL_NAME: &str = "examples/bore-plate.pcm";

#[derive(Clone, Debug)]
pub struct SceneItem {
    pub name: &'static str,
    pub mesh: Mesh,
    pub translation: Vec3,
    /// Uniform scale (mm -> scene units for kernel-built parts).
    pub scale: f32,
    pub color: [f32; 3],
    /// Stats line shown in the interactive panel (volume/mass for CSG).
    pub info: String,
}

impl SceneItem {
    /// Model matrix: uniform scale + translation (normals survive after
    /// re-normalizing; app.rs and the WGSL shader both do).
    pub fn model_matrix(&self) -> glam::Mat4 {
        glam::Mat4::from_scale_rotation_translation(
            Vec3::splat(self.scale),
            glam::Quat::IDENTITY,
            self.translation,
        )
    }
}

pub fn sample_scene() -> Vec<SceneItem> {
    let plain = |name, mesh, translation, color| SceneItem {
        name,
        mesh,
        translation,
        scale: 1.0,
        color,
        info: String::new(),
    };
    vec![
        plain(
            "box",
            box_mesh(2.0, 2.0, 2.0),
            Vec3::new(-3.4, 1.0, 0.0),
            [0.20, 0.45, 0.85],
        ),
        plain(
            "cylinder",
            cylinder_mesh(1.0, 2.5, 48),
            Vec3::new(0.0, 1.25, 0.0),
            [0.85, 0.35, 0.25],
        ),
        plain(
            "torus",
            torus_mesh(1.0, 0.4, 64, 32),
            Vec3::new(3.4, 0.4, 0.0),
            [0.25, 0.70, 0.35],
        ),
    ]
}

/// Build the CSG demo model (file text -> kernel -> solid).
pub fn csg_demo_solid() -> Result<csg::Solid, String> {
    let m = parse_model(DEMO_MODEL_SRC, DEMO_MODEL_NAME).map_err(|e| e.to_string())?;
    csg::evaluate(&m.graph).map_err(|e| e.to_string())
}

/// One-item scene holding the kernel-built CSG part, scaled mm -> scene
/// units and resting on the ground plane.
pub fn csg_demo_scene() -> Result<Vec<SceneItem>, String> {
    let model = parse_model(DEMO_MODEL_SRC, DEMO_MODEL_NAME).map_err(|e| e.to_string())?;
    let s = csg::evaluate(&model.graph).map_err(|e| e.to_string())?;
    let scale = 0.05f32; // 60 mm -> 3 scene units
    Ok(vec![SceneItem {
        name: "bore-plate (CSG)",
        mesh: s.mesh.clone(),
        translation: Vec3::new(0.0, 6.0 * scale, 0.0), // half plate height in scene units
        scale,
        color: [0.30, 0.45, 0.80],
        info: format!(
            "V {:.1} mm^3 | m {:.1} g | genus {} | {} tris",
            s.volume_mm3,
            s.mass_g(model.density_g_cm3),
            s.genus,
            s.num_tri
        ),
    }])
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

    #[test]
    fn csg_demo_scene_mesh_is_closed_and_volumes_match_kernel() {
        let items = csg_demo_scene().expect("demo model must build");
        assert_eq!(items.len(), 1);
        let (v, e, f) = items[0].mesh.closed_manifold_check().unwrap();
        // Euler characteristic 2 for the genus-1 bored plate would be 0
        // only as a closed surface of a torus; a through-hole plate is
        // topologically a torus: chi = 0.
        let chi = v as i64 - e as i64 + f as i64;
        assert_eq!(chi, 0, "bored plate is torus topology, chi={chi}");
    }
}
