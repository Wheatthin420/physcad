//! Milestone-3 acceptance gates, driven through the *file* pipeline
//! (parse_model -> csg::evaluate -> stl), exactly like the CLI runs them.
//!
//! Volume gate: |kernel volume − analytic volume| < 1e-6 mm^3 (absolute).
//! Measured (this box, 2026-11-24, all five gates):
//!   cyl  r=1 @16384 segs: 1.54e-7   (budget (2pi/n)^2/6*V: 9.2e-8 — matches)
//!   hemi r=1/4 @1024x1024: 2.82e-7
//!   slug cyl∩box @16384:    2.31e-7
//!   ball r=1/4 @1024x1024:  5.65e-7  (the UV-sphere inscribed error runs
//!                                     ~2.3x the naive (pi^2/m^2+..)/6 estimate)
//! Bit-stability: same model evaluated twice -> identical volume bits,
//! identical STL bytes (cross-process stability is checked in
//! docs/MILESTONES.md via the CLI + sha256sum).

use std::f64::consts::PI;

use physcad::csg;
use physcad::model::parse_model;
use physcad::stl::{from_stl_binary, to_stl_binary};

fn build(src: &str) -> csg::Solid {
    let m = parse_model(src, "gate.pcm").expect("gate model must parse");
    csg::evaluate(&m.graph).expect("gate model must build")
}

fn require_gate(label: &str, got: f64, analytic: f64) {
    let err = (got - analytic).abs();
    eprintln!("GATE {label}: got {got:.9}  analytic {analytic:.9}  |err| {err:.3e}");
    assert!(err < 1e-6, "{label}: |err| {err:.3e} exceeds 1e-6 mm^3");
}

#[test]
fn union_box_cylinder_analytic() {
    // plate 30x5x30, cylinder r=1 h=2 standing on the top face (coplanar
    // contact, zero overlap): analytic union = 4500 + pi*r^2*h
    let s = build(
        "
plate = box(30, 5, 30)
cyl   = cylinder(1, 2, 16384)
cyl_up = move(cyl, 0, 3.5, 0)
part  = union(plate, cyl_up)
",
    );
    require_gate("union box+cyl", s.volume_mm3, 4500.0 + PI * 2.0);
    assert_eq!(s.genus, 0);
}

#[test]
fn cut_box_minus_cylinder_bore_analytic() {
    // plate 10x2x10 with through-bore r=1 (bore h=4 protrudes both faces)
    let s = build(
        "
plate = box(10, 2, 10)
bore  = cylinder(1, 4, 16384)
part  = cut(plate, bore)
",
    );
    require_gate("cut cyl bore", s.volume_mm3, 200.0 - PI * 2.0);
    assert_eq!(s.genus, 1);
}

#[test]
fn intersect_box_cylinder_analytic() {
    // box 3^3 ∩ cylinder r=1 (cyl h=5 fully pierces): analytic = pi*1^2*3
    let s = build(
        "
boxy = box(3, 3, 3)
cyl  = cylinder(1, 5, 16384)
slug = intersect(boxy, cyl)
",
    );
    require_gate("intersect box∩cyl", s.volume_mm3, PI * 3.0);
}

#[test]
fn union_box_sphere_analytic() {
    // unit cube + sphere r=1/4 centered exactly on the top face plane:
    // analytic union = 1 + (2/3)*pi*r^3 (exact half-ball added)
    let s = build(
        "
cube = box(1, 1, 1)
sph  = sphere(0.25, 1024, 1024)
sph_on = move(sph, 0, 0.5, 0)
part = union(cube, sph_on)
",
    );
    let half_ball = 2.0 / 3.0 * PI * 0.25f64.powi(3);
    require_gate("union box+hemisphere", s.volume_mm3, 1.0 + half_ball);
}

#[test]
fn cut_box_sphere_cavity_analytic() {
    // unit cube minus fully-interior sphere r=1/4: analytic = 1 - (4/3)pi r^3
    let s = build(
        "
cube = box(1, 1, 1)
sph  = sphere(0.25, 1024, 1024)
part = cut(cube, sph)
",
    );
    let ball = 4.0 / 3.0 * PI * 0.25f64.powi(3);
    require_gate("cut spherical cavity", s.volume_mm3, 1.0 - ball);
    // manifold genus convention (verified): a solid with an INTERIOR CAVITY
    // has two boundary shells and reports genus = -1, same as a
    // disconnected union — through-holes count +1, extra shells count -1.
    assert_eq!(s.genus, -1, "interior cavity = extra closed shell");
}

#[test]
fn scripted_rebuild_is_bit_stable() {
    // A mixed tree (params + expressions + all three ops) built twice in
    // this process: volume bit-identical, mesh equal, STL byte-identical.
    const SRC: &str = "
density = 2.7
param plate_w = 40
param t = plate_w / 10              # 4
param bore_r = t * 1.5              # 6
plate  = box(plate_w, t, plate_w/2) # 40x4x20
bore   = cylinder(bore_r, t + 2, 256)
knob   = sphere(t, 64, 128)
knob_h = move(knob, plate_w/4, t/2, 0)
stem   = union(plate, knob_h)
part   = cut(stem, bore)
plug   = intersect(knob, plate)    # exercises intersect (a no-op union below)
final  = union(stem, plug)
";
    let m = parse_model(SRC, "stable.pcm").unwrap();
    let a = csg::evaluate(&m.graph).unwrap();
    let b = csg::evaluate(&m.graph).unwrap();
    assert_eq!(a.volume_mm3.to_bits(), b.volume_mm3.to_bits());
    assert_eq!(a.mesh, b.mesh);
    let sa = to_stl_binary(&a.mesh);
    let sb = to_stl_binary(&b.mesh);
    assert_eq!(sa.len(), sb.len());
    assert_eq!(sa, sb, "STL bytes must be identical across rebuilds");
    eprintln!(
        "bit-stable: {} tris, {} STL bytes, V={:.9}",
        a.num_tri,
        sa.len(),
        a.volume_mm3
    );

    // The exported STL must survive a round trip: parse it back and the
    // f32 mesh must still carry the kernel volume to within f32
    // quantization (the analytic gates above run on the f64 kernel value).
    // f32 STL *quantization* (sum in f64 to isolate it from f32
    // accumulation) must cost less than 1e-6 relative volume.
    let back = from_stl_binary(&sa).unwrap();
    assert_eq!(back.triangle_count(), a.num_tri);
    let vol64: f64 = back
        .indices
        .chunks(3)
        .map(|t| {
            let p: [glam::DVec3; 3] =
                std::array::from_fn(|k| glam::Vec3::from(back.positions[t[k] as usize]).as_dvec3());
            p[0].dot(p[1].cross(p[2])) / 6.0
        })
        .sum();
    let rel = (vol64 - a.volume_mm3).abs() / a.volume_mm3;
    assert!(
        rel < 1e-6,
        "STL round trip drifted: rel {rel:.3e} ({vol64} vs {})",
        a.volume_mm3
    );
    eprintln!("stl round trip: relative volume drift {rel:.3e}");
}
