//! Milestone-3 render gate: the *CSG kernel output* (bore-plate.pcm ->
//! manifold booleans -> mesh) must render headless through the real wgpu
//! pipeline, not just exist as data. Also pins the CSG mesh's kernel
//! volume against an independently computed analytic expectation.

use std::path::Path;

use physcad::model::parse_model;
use physcad::stl::{from_stl_binary, to_stl_binary};

#[test]
fn csg_demo_part_renders_headless() {
    let (w, h) = (640u32, 360u32);
    let out = concat!(env!("CARGO_TARGET_TMPDIR"), "/physcad-m3-csg-capture.png");
    let scene = physcad_view::scene::csg_demo_scene().expect("CSG demo scene must build");
    let report = physcad_view::renderer::render_scene_png(&scene, Path::new(out), w, h)
        .expect("offscreen capture must succeed (Vulkan ICD; lavapipe is the software fallback)");
    eprintln!(
        "adapter: {} ({} / {})",
        report.adapter_name, report.adapter_device_type, report.adapter_backend
    );

    let img = image::open(out).expect("written PNG must decode");
    assert_eq!(img.width(), w);
    assert_eq!(img.height(), h);
    let rgba = img.to_rgba8();
    let bg = *rgba.get_pixel(0, 0);

    // The CSG part is steel-blue (0.30, 0.45, 0.80 * Lambert): count its
    // pixels and require a believable silhouette near the frame center.
    // The kernel torus ring is copper (0.80, 0.52, 0.25 * Lambert) and
    // must also show up — proof the torus path renders, not just builds.
    let (mut blue, mut copper, mut total) = (0u32, 0u32, 0u32);
    let mut xs = Vec::new();
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
            xs.push(x);
            let _ = y;
        }
        if r as i16 > b as i16 + 40 && r as i16 >= g as i16 + 20 && g as i16 > b as i16 {
            copper += 1;
        }
    }
    let span = xs.iter().max().unwrap() - xs.iter().min().unwrap();
    eprintln!("off-bg px: {total}, CSG blue px: {blue}, copper px: {copper}, x-span: {span}");
    assert!(
        blue > 2000,
        "CSG part must cover a solid silhouette: {blue} px"
    );
    assert!(copper > 200, "kernel torus ring must render: {copper} px");
    assert!(
        (100..400).contains(&span),
        "part should span a plausible width (got {span} px)"
    );
    assert!(
        report.distinct_colors > 5,
        "shading must produce multiple tones"
    );
}

#[test]
fn csg_mesh_volume_matches_analytic_and_stl_round_trips() {
    // Exact expected volume of the demo part AT ITS FILE TESSELLATION:
    //   plate(60x12x40) - inscribed-96gon prism(r=5, h=12)
    //   + UV-sphere polyhedron half (r=6, rings=48, segs=96), cut exactly
    //     at its equator row (moves put the equator on the plate face y=6).
    // The UV polyhedron band between latitudes is a prismatoid between
    // parallel n-gons (faces are planar quads), so the prismoid formula
    // V=(dy/6)(A_j + 4 A_mid + A_{j+1}) with A=(n/2) rho^2 sin(2pi/n) is
    // exact — no analytic-sphere slop in this test.
    use std::f64::consts::PI;
    let model = parse_model(
        physcad_view::scene::DEMO_MODEL_SRC,
        physcad_view::scene::DEMO_MODEL_NAME,
    )
    .unwrap();
    let s = physcad::csg::evaluate(&model.graph).unwrap();
    assert_eq!(s.genus, 1);

    let bore_prism = 0.5 * 96.0 * 25.0 * (2.0 * PI / 96.0).sin() * 12.0;

    let (r, m, n) = (6.0f64, 48usize, 96usize);
    let dphi = PI / m as f64;
    let k = (n as f64 / 2.0) * (2.0 * PI / n as f64).sin(); // area = k*rho^2
    let rho = |j: usize| r * (PI / 2.0 - j as f64 * dphi).cos();
    let y = |j: usize| r * (PI / 2.0 - j as f64 * dphi).sin();
    let mut hemi_poly = 0.0;
    for j in 0..m / 2 {
        // north pole (j=0) has rho_0 = 0, formula covers the cap fan too
        let dy = (y(j) - y(j + 1)).abs();
        let amid = k * 0.25 * (rho(j) + rho(j + 1)).powi(2); // ((rj+rk)/2)^2
        hemi_poly += dy / 6.0 * (k * rho(j).powi(2) + 4.0 * amid + k * rho(j + 1).powi(2));
    }
    let exact_poly = 28800.0 - bore_prism + hemi_poly;
    let dev = (s.volume_mm3 - exact_poly).abs();
    eprintln!(
        "demo: kernel V={:.6}; exact-polyhedral={:.6}; dev {dev:.3e}",
        s.volume_mm3, exact_poly
    );
    assert!(dev < 1e-6, "kernel vs its own tessellation: {dev:e}");

    // STL export -> parse back -> triangle count identical, volume stable.
    // Volume of the re-read mesh is summed in f64: Mesh::signed_volume is
    // an f32 accumulator and its own rounding (~1e-7 relative * n terms)
    // would swamp the quantization effect we are measuring here.
    let bytes = to_stl_binary(&s.mesh);
    let back = from_stl_binary(&bytes).unwrap();
    assert_eq!(back.triangle_count(), s.num_tri);
    let vol64: f64 = back
        .indices
        .chunks(3)
        .map(|t| {
            let p: [glam::DVec3; 3] =
                std::array::from_fn(|k| glam::Vec3::from(back.positions[t[k] as usize]).as_dvec3());
            p[0].dot(p[1].cross(p[2])) / 6.0
        })
        .sum();
    let rel = (vol64 - s.volume_mm3).abs() / s.volume_mm3;
    eprintln!(
        "stl round trip: kernel {v:.6} -> f32 stl -> f64 read {vol64:.6}, rel drift {rel:.3e}",
        v = s.volume_mm3
    );
    assert!(rel < 1e-5, "STL f32 quantization drift rel {rel:.3e}");
}
