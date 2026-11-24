//! Milestone-2 gate: draw box + cylinder + torus offscreen (no display
//! server) and write a PNG. The assertions check that each third of the
//! frame really contains the geometry expected there — not just "a file
//! appeared".

use std::path::Path;

#[derive(Default, Clone, Copy, Debug)]
struct HueCounts {
    blue: u32,
    red: u32,
    green: u32,
    off_background: u32,
}

#[test]
fn headless_capture_renders_all_three_primitives() {
    let (w, h) = (640u32, 360u32);
    let out = concat!(env!("CARGO_TARGET_TMPDIR"), "/physcad-m2-capture.png");
    let report = physcad_view::renderer::capture_png(Path::new(out), w, h)
        .expect("offscreen capture must succeed (Vulkan ICD; lavapipe is the software fallback)");
    eprintln!(
        "adapter: {} ({} / {})",
        report.adapter_name, report.adapter_device_type, report.adapter_backend
    );

    // File exists and decodes at the requested size.
    let img = image::open(out).expect("written PNG must decode");
    assert_eq!(img.width(), w);
    assert_eq!(img.height(), h);
    let rgba = img.to_rgba8();
    let bg = *rgba.get_pixel(0, 0); // corner is empty background

    // Bucket hues per horizontal third: left=box(blue), center=cylinder(red),
    // right=torus(green). Thresholds are loose but far from the pale background.
    let mut thirds = [HueCounts::default(); 3];
    for (x, _y, px) in rgba.enumerate_pixels() {
        let [r, g, b, _] = px.0;
        if (r as i16 - bg.0[0] as i16).abs()
            + (g as i16 - bg.0[1] as i16).abs()
            + (b as i16 - bg.0[2] as i16).abs()
            < 30
        {
            continue; // background
        }
        let t = ((x * 3) / w).min(2) as usize;
        thirds[t].off_background += 1;
        if b as i16 > r as i16 + 20 && b as i16 > g as i16 + 10 {
            thirds[t].blue += 1;
        } else if r as i16 > g as i16 + 20 && r as i16 > b as i16 + 20 {
            thirds[t].red += 1;
        } else if g as i16 > r as i16 + 20 && g as i16 > b as i16 + 10 {
            thirds[t].green += 1;
        }
    }

    assert!(
        thirds[0].blue > 500,
        "expected blue box pixels in left third, got {:?}",
        thirds[0]
    );
    assert!(
        thirds[1].red > 500,
        "expected red cylinder pixels in center third, got {:?}",
        thirds[1]
    );
    assert!(
        thirds[2].green > 500,
        "expected green torus pixels in right third, got {:?}",
        thirds[2]
    );
    // Nothing should be drawn in a fourth place; background dominates.
    let object_pixels: u32 = thirds.iter().map(|t| t.off_background).sum();
    assert!(
        object_pixels < w * h / 2,
        "scene occupies implausibly much of the frame: {object_pixels} px"
    );
}
