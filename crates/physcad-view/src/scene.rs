//! Scene contents for both render paths.
//!
//! M2 sample (box/cylinder/torus primitives) plus the M3 CSG demo: the
//! bore-plate model file is *built by the kernel* (model file -> params ->
//! CSG tree -> manifold booleans) and its resulting mesh is what the
//! viewer draws — the same `physcad::csg::evaluate` the CLI runs.
//! M4 adds the solved-sketch demo: sketch entities -> LM solver -> curve
//! polylines drawn as flat ribbons on the ground plane.
//! Y-up world, primitives resting on the ground plane y=0.

use glam::Vec3;
use physcad::csg;
use physcad::model::parse_model;
use physcad::primitives::{Mesh, box_mesh, cylinder_mesh, torus_mesh};
use physcad::sketch::solver::{SolveStatus, solve};

pub const DEMO_MODEL_SRC: &str = include_str!("../../../examples/bore-plate.pcm");
pub const DEMO_MODEL_NAME: &str = "examples/bore-plate.pcm";
pub const SKETCH_MODEL_SRC: &str = include_str!("../../../examples/sketch-bracket.pcm");
pub const SKETCH_MODEL_NAME: &str = "examples/sketch-bracket.pcm";

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

/// Kernel torus for the render gate — built by the same file grammar, at
/// *render* tessellation (96x48; the 1e-6 analytic gates for tori live in
/// crates/physcad/tests/m3_gates.rs).
pub const TORUS_DEMO_SRC: &str = "ring = torus(8, 2, 96, 48)\n";

pub fn torus_demo_solid() -> Result<csg::Solid, String> {
    let m = parse_model(TORUS_DEMO_SRC, "render-torus.pcm").map_err(|e| e.to_string())?;
    csg::evaluate(&m.graph).map_err(|e| e.to_string())
}

/// Kernel-built parts: the bore-plate plus a standing torus ring, scaled
/// mm -> scene units and resting on the ground plane.
pub fn csg_demo_scene() -> Result<Vec<SceneItem>, String> {
    let model = parse_model(DEMO_MODEL_SRC, DEMO_MODEL_NAME).map_err(|e| e.to_string())?;
    let s = csg::evaluate(&model.graph).map_err(|e| e.to_string())?;
    let t = torus_demo_solid()?;
    let scale = 0.05f32; // 60 mm -> 3 scene units
    Ok(vec![
        SceneItem {
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
        },
        SceneItem {
            name: "torus ring (CSG)",
            mesh: t.mesh,
            // R=8, tube=2 mm -> outer dia 20 mm -> 1.0 scene unit; center at
            // tube radius so the ring rests flat on the ground plane.
            translation: Vec3::new(2.4, 2.0 * scale, 2.2),
            scale,
            color: [0.80, 0.52, 0.25],
            info: format!(
                "V {:.1} mm^3 | genus {} | {} tris",
                t.volume_mm3, t.genus, t.num_tri
            ),
        },
    ])
}

/// Flat ribbon for one polyline of the solved sketch: quads on the y=lift
/// plane with +Y normals (winding derived from a rule, M2 lesson).
fn ribbon_mesh(points: &[Vec3], hw: f32, lift: f32, out: &mut Mesh) {
    for seg in points.windows(2) {
        let (a, b) = (seg[0], seg[1]);
        let d = b - a;
        // perp = normalize(cross(dir, up)) * hw; degenerate segments skipped
        let perp = d.cross(Vec3::Y);
        if perp.length_squared() < 1e-12 {
            continue;
        }
        let perp = perp.normalize_or_zero() * hw;
        let (q1, q2) = (a + perp, b + perp);
        let (q3, q4) = (b - perp, a - perp);
        for (v0, v1, v2) in [(q1, q2, q3), (q1, q3, q4)] {
            let n = (v1 - v0).cross(v2 - v0);
            let n = if n.dot(Vec3::Y) < 0.0 { -n } else { n };
            let n = n.normalize_or_zero();
            let base = out.positions.len() as u32;
            for v in [v0, v1, v2] {
                out.positions.push([v.x, lift, v.z]);
                out.normals.push(n.to_array());
            }
            out.indices.extend_from_slice(&[base, base + 1, base + 2]);
        }
    }
}

/// Small hexagon dot marking a sketch point, +Y normal, CCW from above.
fn dot_mesh(center: Vec3, r: f32, lift: f32, out: &mut Mesh) {
    let c = out.positions.len() as u32;
    out.positions.push([center.x, lift, center.z]);
    out.normals.push(Vec3::Y.to_array());
    for k in 0..6 {
        let a = std::f32::consts::TAU * k as f32 / 6.0;
        out.positions
            .push([center.x + r * a.cos(), lift, center.z + r * a.sin()]);
        out.normals.push(Vec3::Y.to_array());
    }
    for k in 0..6u32 {
        // world XZ plane: increasing angle is CW seen from +Y, so the
        // fan must be (center, v[k+1], v[k]) to face up.
        out.indices
            .extend_from_slice(&[c, c + 1 + (k + 1) % 6, c + 1 + k]);
    }
}

/// Build the M4 sketch demo scene: parse the .pcm sketch, solve it with
/// the LM solver, tessellate the *solved* entities, and lay the curves
/// flat on the ground plane (sketch +x -> world +x, +y -> world -z).
pub fn sketch_demo_scene() -> Result<Vec<SceneItem>, String> {
    let model = parse_model(SKETCH_MODEL_SRC, SKETCH_MODEL_NAME).map_err(|e| e.to_string())?;
    let sk = model
        .sketches
        .first()
        .ok_or("demo sketch file declares no sketch")?;
    let (solved, rep) = solve(sk);
    if rep.status != SolveStatus::Solved {
        return Err(format!("demo sketch must solve: {rep}"));
    }
    let scale = 0.05f32; // 80 mm -> 4 scene units
    let polylines = solved.polylines(0.02);
    // center the sketch bbox on the world origin
    let pts: Vec<[f64; 2]> = polylines.iter().flat_map(|p| p.iter().copied()).collect();
    let minx = pts.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let maxx = pts.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
    let miny = pts.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
    let maxy = pts.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
    let (cx, cy) = ((minx + maxx) / 2.0, (miny + maxy) / 2.0);
    let world = |p: [f64; 2]| {
        Vec3::new(
            ((p[0] - cx) * scale as f64) as f32,
            0.0,
            (-(p[1] - cy) * scale as f64) as f32,
        )
    };
    let lift = 0.01f32;
    let info = format!(
        "max|res| {:.3e} | rank {}/{} | dof {} | {} iters",
        rep.max_residual, rep.rank, rep.n_params, rep.dof, rep.iterations
    );
    let mut items = Vec::new();
    for (i, pl) in polylines.iter().enumerate() {
        let mut mesh = Mesh::new();
        let entity_name = solved.entities[i].0.as_str();
        let is_point = pl.len() == 1;
        let color = if is_point {
            dot_mesh(world(pl[0]), 0.075, lift, &mut mesh);
            [0.80, 0.20, 0.20]
        } else {
            let wpts: Vec<Vec3> = pl.iter().map(|p| world(*p)).collect();
            ribbon_mesh(&wpts, 0.050, lift, &mut mesh);
            if entity_name == "fillet" || entity_name == "qa" {
                [0.85, 0.55, 0.15]
            } else {
                [0.10, 0.30, 0.45]
            }
        };
        items.push(SceneItem {
            name: if is_point {
                "sketch point"
            } else {
                "sketch curve"
            },
            mesh,
            translation: Vec3::ZERO,
            scale: 1.0,
            color,
            info: if items.is_empty() {
                info.clone()
            } else {
                String::new()
            },
        });
    }
    Ok(items)
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
        assert_eq!(items.len(), 2);
        for item in &items {
            let (v, e, f) = item.mesh.closed_manifold_check().unwrap();
            let chi = v as i64 - e as i64 + f as i64;
            // Bored plate (genus 1 through-hole) and solid torus ring both
            // have single-shell chi = 0.
            assert_eq!(chi, 0, "{}: single genus-1 shell", item.name);
        }
        // Kernel torus vs analytic 2*pi^2*R*r^2 at RENDER tessellation
        // (96x48): inscribed-polygon slop is ~1e-3 relative here — the
        // 1e-6 analytic gates run in crates/physcad/tests/m3_gates.rs.
        let t = torus_demo_solid().unwrap();
        let analytic = 2.0 * std::f64::consts::PI * std::f64::consts::PI * 8.0 * 2.0 * 2.0;
        let rel = (t.volume_mm3 - analytic).abs() / analytic;
        eprintln!(
            "render torus: kernel V={:.4} analytic {analytic:.4} rel {rel:.3e}, genus {}",
            t.volume_mm3, t.genus
        );
        assert!(rel < 2e-2, "render-tessellation torus rel {rel:.3e}");
        assert_eq!(t.genus, 1, "kernel torus is genus 1");
    }

    #[test]
    fn sketch_demo_scene_all_face_normals_point_up() {
        let items = sketch_demo_scene().expect("sketch demo must solve and build");
        // 8 points + 4 rectangle lines + 1 fillet circle + 1 quarter arc
        assert_eq!(items.len(), 14);
        let mut tris = 0usize;
        for item in &items {
            for tri in item.mesh.indices.chunks(3) {
                let p = [0, 1, 2].map(|k| glam::Vec3::from(item.mesh.positions[tri[k] as usize]));
                let n = (p[1] - p[0]).cross(p[2] - p[0]);
                assert!(
                    n.dot(Vec3::Y) > 0.0,
                    "{}: ribbon triangle faces down",
                    item.name
                );
                let stored = item.mesh.normals[tri[0] as usize];
                assert!(
                    (stored[1] - 1.0).abs() < 1e-3
                        && stored[0].abs() < 1e-3
                        && stored[2].abs() < 1e-3,
                    "{}: stored normal {stored:?}",
                    item.name
                );
                tris += 1;
            }
        }
        eprintln!("sketch demo: {} items, {tris} triangles", items.len());
        // 55-seg circle + 13-seg arc + 4 line quads + 8 hex dots = 192
        assert_eq!(tris, 192, "tessellation count must match chord tol");
    }
}
