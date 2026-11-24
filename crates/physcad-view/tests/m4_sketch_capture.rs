//! Milestone-4 render gate: the sketch must solve through the *file*
//! path (.pcm text -> parser -> LM solver) with residual < 1e-9, and the
//! solved curves must render headless through the real wgpu pipeline —
//! counted as pixels, not eyeballed.

use std::path::Path;

use physcad::model::parse_model;
use physcad::sketch::solver::{SolveStatus, solve};

#[test]
fn sketch_solves_from_file_and_renders_headless() {
    // (a) solve gate straight from the .pcm text
    let model = parse_model(
        physcad_view::scene::SKETCH_MODEL_SRC,
        physcad_view::scene::SKETCH_MODEL_NAME,
    )
    .unwrap();
    let (_solved, rep) = solve(&model.sketches[0]);
    eprintln!(
        "solve gate: status={:?} max|res|={:.3e} l2={:.3e} rank={}/{} dof={} iters={}",
        rep.status,
        rep.max_residual,
        rep.l2_residual,
        rep.rank,
        rep.n_params,
        rep.dof,
        rep.iterations
    );
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < 1e-9, "max residual {}", rep.max_residual);
    assert_eq!(rep.dof, 0);
    assert!(rep.redundant.is_empty());

    // (b) render gate
    let (w, h) = (640u32, 360u32);
    let out = concat!(
        env!("CARGO_TARGET_TMPDIR"),
        "/physcad-m4-sketch-capture.png"
    );
    let scene = physcad_view::scene::sketch_demo_scene().expect("sketch demo scene must build");
    let report = physcad_view::renderer::render_scene_png(&scene, Path::new(out), w, h)
        .expect("offscreen capture must succeed (Vulkan ICD; lavapipe is the software fallback)");
    eprintln!(
        "adapter: {} ({} / {})",
        report.adapter_name, report.adapter_device_type, report.adapter_backend
    );

    let img = image::open(out).expect("written PNG must decode");
    assert_eq!((img.width(), img.height()), (w, h));
    let rgba = img.to_rgba8();
    let bg = *rgba.get_pixel(0, 0);

    let near = |px: &image::Rgba<u8>, c: [u8; 3]| -> bool {
        (0..3).all(|k| (px.0[k] as i16 - c[k] as i16).abs() <= 24)
    };
    // Lambert-scaled sketch colors, measured from a capture run on this
    // box (teal curves (89,148,178), amber fillet+arc (236,194,107),
    // red point markers (230,123,123)) — tolerance band 26 per channel.
    let (mut teal, mut amber, mut red, mut offbg) = (0u32, 0u32, 0u32, 0u32);
    let mut xs = Vec::new();
    for (x, _y, px) in rgba.enumerate_pixels() {
        if (px.0[0] as i16 - bg.0[0] as i16).abs()
            + (px.0[1] as i16 - bg.0[1] as i16).abs()
            + (px.0[2] as i16 - bg.0[2] as i16).abs()
            < 30
        {
            continue;
        }
        offbg += 1;
        if near(px, [89, 148, 178]) {
            teal += 1;
            xs.push(x);
        } else if near(px, [236, 194, 107]) {
            amber += 1;
            xs.push(x);
        } else if near(px, [230, 123, 123]) {
            red += 1;
            xs.push(x);
        }
    }
    let span = xs.iter().max().unwrap() - xs.iter().min().unwrap();
    eprintln!("off-bg px: {offbg}, teal: {teal}, amber: {amber}, red dots: {red}, x-span: {span}");
    assert!(teal > 900, "rectangle ribbons must cover pixels: {teal}");
    assert!(
        amber > 350,
        "fillet circle + arc must cover pixels: {amber}"
    );
    assert!(red > 60, "point markers must cover pixels: {red}");
    assert!(
        (150..500).contains(&span),
        "sketch should span a plausible width (got {span} px)"
    );
    assert!(
        report.distinct_colors >= 4,
        "flat ribbons give bg + 3 curve tones (no MSAA): got {}",
        report.distinct_colors
    );
}
