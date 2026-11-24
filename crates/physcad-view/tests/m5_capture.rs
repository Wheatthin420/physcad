//! Milestone-5 render gate: an EXTRUDE+CSG composition (sketch-plate.pcm:
//! solved sketch -> extrude -> cut pocket) and a REVOLVED bushing must
//! render headless through the real wgpu pipeline and survive an STL
//! round trip — counted as pixels and bytes, not eyeballed.
//!
//! M3 lesson applied: the capture pipeline uses cull_mode None + depth
//! test (flat caps of extruded solids draw nothing under Front culling
//! with our CCW convention + y-down framebuffer).

use std::path::Path;

use physcad::model::parse_model;
use physcad::stl::{from_stl_binary, to_stl_binary};

#[test]
fn m5_extrude_csg_composition_renders_headless() {
    let (w, h) = (640u32, 360u32);
    let out = concat!(
        env!("CARGO_TARGET_TMPDIR"),
        "/physcad-m5-extrude-csg-capture.png"
    );
    let scene = physcad_view::scene::m5_scene().expect("M5 scene must build");
    // steep elevation: the through-bore reads as background in the top
    // face (no floor geometry in this scene; sky shows through).
    let camera = physcad::camera::OrbitCamera {
        target: glam::Vec3::new(0.0, 0.35, 0.0),
        distance: 7.0,
        yaw: 0.9,
        pitch: 1.1,
        ..Default::default()
    };
    let report =
        physcad_view::renderer::render_scene_png_with_camera(&scene, Path::new(out), w, h, camera)
            .expect(
                "offscreen capture must succeed (Vulkan ICD; lavapipe is the software fallback)",
            );
    eprintln!(
        "adapter: {} ({} / {})",
        report.adapter_name, report.adapter_device_type, report.adapter_backend
    );

    let img = image::open(out).expect("written PNG must decode");
    assert_eq!((img.width(), img.height()), (w, h));
    let rgba = img.to_rgba8();
    let bg = *rgba.get_pixel(0, 0);

    // same tone classes as the M3 gate: steel-blue plate, copper ring
    let (mut blue, mut copper, mut total) = (0u32, 0u32, 0u32);
    let mut plate_box = (w, h, 0u32, 0u32); // minx,miny,maxx,maxy of blue
    for (x, y, px) in rgba.enumerate_pixels() {
        let [r, g, b, _] = px.0;
        if (r as i16 - bg.0[0] as i16).abs()
            + (g as i16 - bg.0[1] as i16).abs()
            + (b as i16 - bg.0[2] as i16).abs()
            < 30
        {
            continue;
        }
        total += 1;
        if b as i16 > r as i16 + 20 && b as i16 > g as i16 + 10 {
            blue += 1;
            plate_box.0 = plate_box.0.min(x);
            plate_box.1 = plate_box.1.min(y);
            plate_box.2 = plate_box.2.max(x);
            plate_box.3 = plate_box.3.max(y);
        }
        if r as i16 > b as i16 + 40 && r as i16 >= g as i16 + 20 && g as i16 > b as i16 {
            copper += 1;
        }
    }
    eprintln!(
        "off-bg px: {total}, plate blue px: {blue}, bushing copper px: {copper}, \
         plate bbox: {plate_box:?}"
    );
    assert!(blue > 2000, "extrude+CSG plate must cover pixels: {blue}");
    assert!(copper > 200, "revolved bushing must render: {copper}");
    assert!(report.distinct_colors > 5, "shading must produce tones");

    // the sketch bore must see THROUGH the plate: anchor the probe
    // window on the PROJECTED bore center (sketch origin -> world
    // (0, plate_top, 0) through the same view matrix the renderer used);
    // the bore is the only hole in that window (the corner pocket sits
    // in a bbox corner, not the middle).
    let aspect = w as f32 / h as f32;
    // glam's project_point3 already divides by w
    let p = camera
        .view_projection(aspect)
        .project_point3(glam::Vec3::new(0.0, 0.6, 0.0));
    let sx = (p.x * 0.5 + 0.5) * w as f32;
    let sy = (1.0 - (p.y * 0.5 + 0.5)) * h as f32;
    assert!(
        (0.0..w as f32).contains(&sx) && (0.0..h as f32).contains(&sy),
        "bore center must project inside the frame: ({sx},{sy})"
    );
    let rx = 26i32;
    let ry = 20i32;
    let (cx, cy) = (sx as i32, sy as i32);
    let mut see_through = 0u32;
    let mut window_px = 0u32;
    for y in cy - ry..=cy + ry {
        for x in cx - rx..=cx + rx {
            window_px += 1;
            let px = rgba.get_pixel(x as u32, y as u32);
            let [r, g, b, _] = px.0;
            let is_bg = (r as i16 - bg.0[0] as i16).abs()
                + (g as i16 - bg.0[1] as i16).abs()
                + (b as i16 - bg.0[2] as i16).abs()
                < 30;
            if is_bg {
                see_through += 1;
            }
        }
    }
    eprintln!(
        "bore projects to ({sx:.1},{sy:.1}); window {window_px} px, background inside \
         (bore sees through): {see_through}"
    );
    assert!(
        (120..1600).contains(&see_through),
        "through-bore must be visible in the bore window: {see_through} px"
    );

    // the revolved bushing renders as a hollow tube: its bbox center
    // window must contain non-copper pixels (we look through the bore)
    let mut cb = (w, h, 0u32, 0u32);
    for (x, y, px2) in rgba.enumerate_pixels() {
        let [r, g, b, _] = px2.0;
        if r as i16 > b as i16 + 40 && r as i16 >= g as i16 + 20 && g as i16 > b as i16 {
            cb.0 = cb.0.min(x);
            cb.1 = cb.1.min(y);
            cb.2 = cb.2.max(x);
            cb.3 = cb.3.max(y);
        }
    }
    let bcx = (cb.0 + cb.2) / 2;
    let bcy = (cb.1 + cb.3) / 2;
    let brx = (cb.2 - cb.0) / 6;
    let bry = (cb.3 - cb.1) / 6;
    let mut tube_open = 0u32;
    for y in bcy - bry..=bcy + bry {
        for x in bcx - brx..=bcx + brx {
            let [r, g, b, _] = rgba.get_pixel(x, y).0;
            let is_copper =
                r as i16 > b as i16 + 40 && r as i16 >= g as i16 + 20 && g as i16 > b as i16;
            if !is_copper {
                tube_open += 1;
            }
        }
    }
    eprintln!("bushing bbox {cb:?}: non-copper px in center window (tube open): {tube_open}");
    assert!(tube_open > 40, "bushing tube must be open: {tube_open} px");
}

#[test]
fn m5_demo_stl_export_round_trips_and_is_bit_stable() {
    let m = parse_model(
        physcad_view::scene::M5_MODEL_SRC,
        physcad_view::scene::M5_MODEL_NAME,
    )
    .unwrap();
    let s1 = physcad::csg::evaluate_with(&m.graph, &m.sketches).unwrap();
    let s2 = physcad::csg::evaluate_with(&m.graph, &m.sketches).unwrap();
    let b1 = to_stl_binary(&s1.mesh);
    let b2 = to_stl_binary(&s2.mesh);
    assert_eq!(b1, b2, "sketch->solid pipeline must be bit-stable");
    let back = from_stl_binary(&b1).unwrap();
    assert_eq!(back.triangle_count(), s1.num_tri);
    let vol64: f64 = back
        .indices
        .chunks(3)
        .map(|t| {
            let p = [0, 1, 2].map(|k| {
                let q = back.positions[t[k] as usize];
                glam::DVec3::new(q[0] as f64, q[1] as f64, q[2] as f64)
            });
            p[0].cross(p[1]).dot(p[2]) / 6.0
        })
        .sum();
    let rel = (vol64 - s1.volume_mm3).abs() / s1.volume_mm3;
    eprintln!(
        "m5 STL export: {} bytes, kernel V={:.6}, STL-reread V(f64 sum)={:.6}, rel {rel:.3e}",
        b1.len(),
        s1.volume_mm3,
        vol64,
    );
    assert!(rel < 1e-8, "STL quantization rel {rel:.3e}");
}
