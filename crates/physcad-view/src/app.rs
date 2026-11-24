//! Interactive eframe/egui shell: orbit with drag, zoom with scroll,
//! primitive toggles in a side panel.
//!
//! The 3D scene is drawn by projecting triangles through the *same*
//! `OrbitCamera` matrices the offscreen wgpu renderer uses, then painting
//! them as egui convex polygons (painter's algorithm; good enough for the
//! demo scene — the capture path has a real z-buffer). egui 0.37 has no
//! Mesh3D/Camera3D (grep of the vendored crate finds neither), hence this
//! explicit projection.

use egui::{Color32, Rect, Sense, Shape, Stroke};
use glam::{Mat3, Vec3};
use physcad::camera::OrbitCamera;

use crate::scene::SceneItem;

pub struct ViewerApp {
    camera: OrbitCamera,
    scene: Vec<SceneItem>,
    visible: Vec<bool>,
}

impl ViewerApp {
    pub fn new(scene: Vec<SceneItem>) -> Self {
        let visible = vec![true; scene.len()];
        Self {
            camera: OrbitCamera {
                target: glam::Vec3::new(0.0, 1.1, 0.0),
                ..OrbitCamera::default()
            },
            scene,
            visible,
        }
    }
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::left("controls").show(ui, |ui| {
            ui.heading("PhysCAD — viewer shell");
            ui.separator();
            for (i, item) in self.scene.iter().enumerate() {
                ui.checkbox(&mut self.visible[i], item.name);
            }
            ui.separator();
            if ui.button("reset view").clicked() {
                self.camera = OrbitCamera {
                    target: glam::Vec3::new(0.0, 1.1, 0.0),
                    ..OrbitCamera::default()
                };
            }
            ui.add_space(8.0);
            ui.label("drag: orbit · scroll: zoom");
            ui.label(format!("distance {:.1}", self.camera.distance));
        });

        egui::CentralPanel::default().show(ui, |ui| {
            let rect = ui.available_rect_before_wrap();
            let resp = ui
                .interact(rect, egui::Id::unique("physcad-scene"), Sense::drag())
                .on_hover_cursor(egui::CursorIcon::Grab);
            if resp.dragged() {
                let d = resp.drag_delta();
                self.camera.orbit(d.x, d.y);
            }
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.zoom(scroll);
            }
            paint_scene(ui, rect, &self.camera, &self.scene, &self.visible);
        });
    }
}

/// Lambert shading matching assets/shaders/mesh.wgsl so both paths agree.
fn shade(base: [f32; 3], n: Vec3) -> Color32 {
    let key = Vec3::new(0.4, 0.85, 0.35).normalize();
    let fill = Vec3::new(-0.5, 0.2, -0.6).normalize();
    let diff = n.dot(key).max(0.0) * 0.75 + n.dot(fill).max(0.0) * 0.20;
    let s = 0.30 + diff;
    let q = |v: f32| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
    Color32::from_rgb(q(base[0] * s), q(base[1] * s), q(base[2] * s))
}

fn paint_scene(
    ui: &mut egui::Ui,
    rect: Rect,
    camera: &OrbitCamera,
    scene: &[SceneItem],
    visible: &[bool],
) {
    let aspect = rect.width() / rect.height().max(1.0);
    let view = camera.view();
    let vp = camera.view_projection(aspect);
    let mut tris: Vec<(f32, [egui::Pos2; 3], Color32)> = Vec::new();

    for (item, show) in scene.iter().zip(visible.iter()) {
        if !*show {
            continue;
        }
        let model = item.model_matrix();
        let mvp = vp * model;
        // Model matrices are rigid for now, so the Mat3 block maps normals.
        let normal_matrix = Mat3::from_cols(
            model.x_axis.truncate(),
            model.y_axis.truncate(),
            model.z_axis.truncate(),
        );
        for tri in item.mesh.indices.chunks(3) {
            let p = [0, 1, 2].map(|k| Vec3::from(item.mesh.positions[tri[k] as usize]));
            let n = Vec3::from(item.mesh.normals[tri[0] as usize]);

            // Backface cull in world space against the eye direction.
            let center = (p[0] + p[1] + p[2]) / 3.0;
            let world_center = model.transform_point3(center);
            let world_normal = normal_matrix.mul_vec3(n).normalize();
            if world_normal.dot(camera.eye() - world_center) <= 0.0 {
                continue;
            }

            // Project to screen; skip triangles crossing the near plane.
            let mut ok = true;
            let mut pts = [egui::Pos2::ZERO; 3];
            let mut view_z = 0.0;
            for (k, pos) in p.iter().enumerate() {
                let world = model.transform_point3(*pos);
                let clip = mvp.project_point3(*pos);
                let vpos = view.transform_point3(world);
                view_z += vpos.z;
                if vpos.z > -camera.near {
                    ok = false;
                    break;
                }
                pts[k] = egui::Pos2::new(
                    rect.left() + (clip.x * 0.5 + 0.5) * rect.width(),
                    rect.bottom() - (clip.y * 0.5 + 0.5) * rect.height(),
                );
            }
            if !ok {
                continue;
            }
            tris.push((view_z, pts, shade(item.color, world_normal)));
        }
    }

    // Painter's algorithm: view space looks down -Z, so most-negative
    // (farthest) first; nearer triangles paint over them.
    tris.sort_by(|a, b| a.0.total_cmp(&b.0));
    let painter = ui.painter();
    for (_, pts, color) in tris {
        painter.add(Shape::convex_polygon(pts.to_vec(), color, Stroke::NONE));
    }
}

pub fn run() -> eframe::Result {
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1180.0, 720.0])
            .with_title("PhysCAD — viewer shell (M2)"),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };
    eframe::run_native(
        "physcad-view",
        opts,
        Box::new(|_cc| Ok(Box::new(ViewerApp::new(crate::scene::sample_scene())))),
    )
}
