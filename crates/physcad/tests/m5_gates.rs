//! Milestone-5 gates: closed sketch loops -> solids (extrude/revolve).
//!
//! Precision stance (same as M3): prefer EXACT-POLYHEDRAL checks — the
//! generated mesh IS a polyhedron with a known closed-form volume, so
//! gates run at 1e-9 absolute, no analytic-tessellation slop:
//!   - rectangle extrude            -> box volume (exact)
//!   - N-gon circle extrude         -> A(N-gon) * h, A = (N/2) r^2 sin(2pi/N)
//!   - bore plate (rect + N-gon hole) -> (A_rect - A_N) * h
//!   - rectangle revolve, external axis -> ((n/2) sin(2pi/n) (b^2 - a^2)) * h
//!     full AND partial ((steps/2) sin(dtheta) (b^2 - a^2) h, dtheta = angle/steps)
//!
//! Only the torus-poly gate (circle revolved about an external axis) is a
//! two-fold discretized solid without a clean closed form; it gets a
//! stated relative bound + asserted quadratic convergence (M3 practice).
//!
//! Every solid must pass BOTH closed-manifold checks: the f64 builder
//! mesh before the kernel sees it, and the f32 display mesh after.
//! Euler characteristics are asserted explicitly (sphere=2, torus=0,
//! genus-2 surface=-2, two shells=4).

use std::f64::consts::{PI, TAU};

use physcad::csg::evaluate_with;
use physcad::model::parse_model;
use physcad::sketch::loops::Region2;
use physcad::stl::{from_stl_binary, to_stl_binary};
use physcad::sweep::{Axis2, extrude_region, revolve_region};

fn ngon_area(r: f64, n: usize) -> f64 {
    0.5 * n as f64 * r * r * (TAU / n as f64).sin()
}

/// Euler characteristic of an evaluated solid's display mesh (after the
/// closed-manifold check passes).
fn chi(s: &physcad::csg::Solid) -> i64 {
    let (v, e, f) = s
        .mesh
        .closed_manifold_check()
        .expect("display mesh must be closed");
    v as i64 - e as i64 + f as i64
}

fn build(src: &str, name: &str) -> physcad::csg::Solid {
    let m = parse_model(src, name).expect("model must parse");
    evaluate_with(&m.graph, &m.sketches).expect("model must build")
}

fn rect_sketch(name: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        "{name} = newsketch()\n\
         {name}a = point({name}, {x0}, {y0})\n\
         {name}b = point({name}, {x1}, {y0})\n\
         {name}c = point({name}, {x1}, {y1})\n\
         {name}d = point({name}, {x0}, {y1})\n\
         {name}ab = line({name}, {name}a, {name}b)\n\
         {name}bc = line({name}, {name}b, {name}c)\n\
         {name}cd = line({name}, {name}c, {name}d)\n\
         {name}da = line({name}, {name}d, {name}a)\n"
    )
}

#[test]
fn gate_rectangle_extrude_exact() {
    // direct builder
    let r = Region2 {
        outer: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [0.0, 4.0]],
        holes: vec![],
    };
    let m = extrude_region(&r, 0.0, 6.0).unwrap();
    let (v, e, f) = m.closed_manifold_check().unwrap();
    assert_eq!((v, e, f), (8, 18, 12));
    assert_eq!(v as i64 - e as i64 + f as i64, 2);
    let v64 = m.signed_volume();
    assert!((v64 - 240.0).abs() < 1e-9, "builder got {v64}, exact 240");

    // through the .pcm grammar
    let src = format!(
        "{}\nslab = extrude(s, 6)\n",
        rect_sketch("s", 0.0, 0.0, 10.0, 4.0)
    );
    let s = build(&src, "rect-extrude.pcm");
    let dev = (s.volume_mm3 - 240.0).abs();
    eprintln!(
        "GATE rect-extrude: got {:.9} exact 240 dev {dev:.3e}",
        s.volume_mm3
    );
    assert!(dev < 1e-9, "kernel got {}", s.volume_mm3);
    assert_eq!(s.genus, 0);
    assert_eq!(chi(&s), 2);
}

#[test]
fn gate_circle_approx_extrude_exact_ngon_prism() {
    // circle r=7 at (2,3), sampled as an exact 64-gon, extruded h=5
    let src =
        "s = newsketch()\nc0 = point(s, 2, 3)\nk = circle(s, c0, 7)\nslab = extrude(s, 5, 0, 64)\n";
    let exact = ngon_area(7.0, 64) * 5.0;
    let s = build(src, "circ-extrude.pcm");
    let dev = (s.volume_mm3 - exact).abs();
    eprintln!(
        "GATE circ64-extrude: got {:.9} exact {:.9} dev {dev:.3e}",
        s.volume_mm3, exact
    );
    assert!(dev < 1e-9);
    assert_eq!(s.genus, 0);
    assert_eq!(chi(&s), 2);
}

#[test]
fn gate_bore_plate_from_sketch_exact_with_hole() {
    // rectangle [-10,10] x [-8,8] with an r=5 circle hole, 96-gon bore,
    // extruded 10: exact volume (A_rect - A_96) * 10, genus 1, chi 0.
    let src = format!(
        "{}c = point(s, 0, 0)\nk = circle(s, c, 5)\nslab = extrude(s, 10, 0, 96)\n",
        rect_sketch("s", -10.0, -8.0, 10.0, 8.0)
    );
    let exact = (320.0 - ngon_area(5.0, 96)) * 10.0;
    let s = build(&src, "bore-sketch.pcm");
    let dev = (s.volume_mm3 - exact).abs();
    eprintln!(
        "GATE bore-plate(sketch): got {:.9} exact {:.9} dev {dev:.3e}",
        s.volume_mm3, exact
    );
    assert!(dev < 1e-9);
    assert_eq!(s.genus, 1, "through-bore -> genus 1");
    assert_eq!(chi(&s), 0, "one shell, genus 1: chi 0");
}

#[test]
fn gate_rectangle_revolve_full_exact_shell() {
    // rectangle x in [10,14], y in [0,8] about the (external) vertical
    // line x=0: full 360 deg, 64 sweep steps. The solid is an extruded
    // 64-gon annulus: exact V = (A_64(14) - A_64(10)) * 8.
    let src = format!(
        "{}\
         ax0 = point(s, 0, -2)\nax1 = point(s, 0, 10)\naxl = line(s, ax0, ax1)\n\
         bush = revolve(s, axl, 360, 64)\n",
        rect_sketch("s", 10.0, 0.0, 14.0, 8.0)
    );
    let exact = (ngon_area(14.0, 64) - ngon_area(10.0, 64)) * 8.0;
    let s = build(&src, "revolve-full.pcm");
    let dev = (s.volume_mm3 - exact).abs();
    eprintln!(
        "GATE revolve360 shell: got {:.9} exact {:.9} dev {dev:.3e}",
        s.volume_mm3, exact
    );
    assert!(dev < 1e-9);
    assert_eq!(s.genus, 1, "full revolve of closed profile -> solid torus");
    assert_eq!(chi(&s), 0);

    // and at builder level in f64
    let r = Region2 {
        outer: vec![[10.0, 0.0], [14.0, 0.0], [14.0, 8.0], [10.0, 8.0]],
        holes: vec![],
    };
    let m = revolve_region(
        &r,
        Axis2::through([0.0, -2.0], [0.0, 10.0]).unwrap(),
        TAU,
        64,
    )
    .unwrap();
    let bdev = (m.signed_volume() - exact).abs();
    assert!(bdev < 1e-9, "builder dev {bdev:.3e}");
    let (v, e, f) = m.closed_manifold_check().unwrap();
    assert_eq!(v as i64 - e as i64 + f as i64, 0);
}

#[test]
fn gate_rectangle_revolve_partial90_exact() {
    // 90 deg sweep, 128 steps: every quad spans dtheta = (pi/2)/128;
    // cross-section is a constant polar-quad polygon -> prism:
    // exact V = (steps/2) * sin(dtheta) * (b^2 - a^2) * h
    let steps = 128usize;
    let dtheta = (PI / 2.0) / steps as f64;
    let exact = (steps as f64 / 2.0) * dtheta.sin() * (14.0f64.powi(2) - 10.0f64.powi(2)) * 8.0;
    let r = Region2 {
        outer: vec![[10.0, 0.0], [14.0, 0.0], [14.0, 8.0], [10.0, 8.0]],
        holes: vec![],
    };
    let m = revolve_region(
        &r,
        Axis2::through([0.0, -2.0], [0.0, 10.0]).unwrap(),
        PI / 2.0,
        steps,
    )
    .unwrap();
    let (v, e, f) = m.closed_manifold_check().unwrap();
    assert_eq!(
        v as i64 - e as i64 + f as i64,
        2,
        "partial revolve is a ball: chi 2"
    );
    let dev = (m.signed_volume() - exact).abs();
    eprintln!(
        "GATE revolve90 shell: got {:.9} exact {:.9} dev {dev:.3e} (analytic sector {:.9})",
        m.signed_volume(),
        exact,
        (PI / 4.0) * 96.0 * 8.0
    );
    assert!(dev < 1e-9, "exact-polyhedral dev {dev:.3e}");
    // discretization vs the CONTINUOUS sector shrinks quadratically
    let cont = (PI / 2.0) / 2.0 * 96.0 * 8.0;
    let m32 = revolve_region(
        &r,
        Axis2::through([0.0, -2.0], [0.0, 10.0]).unwrap(),
        PI / 2.0,
        32,
    )
    .unwrap();
    let e32 = (m32.signed_volume() - cont).abs();
    let e128 = (m.signed_volume() - cont).abs();
    eprintln!("GATE revolve90 convergence: e(32) {e32:.3e} -> e(128) {e128:.3e}");
    assert!(e128 * 9.0 < e32, "quadratic convergence expected");
}

#[test]
fn gate_revolve_circle_torus_poly_bounded_and_converging() {
    // circle r=2 centered at x=8 revolved 360 deg about x=0: polyhedral
    // torus (profile 48 x sweep 96). No clean closed form for the doubly
    // discretized polyhedron -> stated relative bound vs 2 pi^2 R r^2,
    // plus quadratic convergence when both resolutions double (M3 rule).
    let src = "s = newsketch()\nc = point(s, 8, 0)\nk = circle(s, c, 2)\n\
               ax0 = point(s, 0, -5)\nax1 = point(s, 0, 5)\naxl = line(s, ax0, ax1)\n\
               ring = revolve(s, axl, 360, 48, 48)\n";
    let analytic = 2.0 * PI * PI * 8.0 * 4.0;
    let s = build(src, "torus-poly.pcm");
    let rel = (s.volume_mm3 - analytic).abs() / analytic;
    eprintln!(
        "GATE torus-poly(48,48): got {:.6} analytic {:.6} rel {rel:.3e}",
        s.volume_mm3, analytic
    );
    assert!(
        rel < 6e-3,
        "torus-poly rel {rel:.3e} outside stated 6e-3 bound"
    );
    assert_eq!(s.genus, 1);
    assert_eq!(chi(&s), 0);

    let src_hi = src.replace("360, 48, 48", "360, 96, 96");
    let s2 = build(&src_hi, "torus-poly-hi.pcm");
    let rel2 = (s2.volume_mm3 - analytic).abs() / analytic;
    eprintln!("GATE torus-poly(96,96): rel {rel2:.3e} (expect ~4x drop)");
    assert!(
        rel2 * 3.0 < rel,
        "quadratic convergence: {rel:.3e} -> {rel2:.3e}"
    );
}

#[test]
fn gate_extrude_then_csg_cut_exact() {
    // composition with the M3 layer: exact bore plate minus a fully
    // interior pocket box: V = (A_rect - A_96) * 10 - 4*3*4 exactly.
    let src = format!(
        "{}c = point(s, 0, 0)\nk = circle(s, c, 5)\nslab = extrude(s, 10, 0, 96)\n\
         pocket = box(4, 3, 4)\npocket_at = move(pocket, -7, -6, 8)\n\
         part = cut(slab, pocket_at)\n",
        rect_sketch("s", -10.0, -8.0, 10.0, 8.0)
    );
    let exact = (320.0 - ngon_area(5.0, 96)) * 10.0 - 48.0;
    let s = build(&src, "extrude-cut.pcm");
    let dev = (s.volume_mm3 - exact).abs();
    eprintln!(
        "GATE extrude+cut: got {:.9} exact {:.9} dev {dev:.3e}",
        s.volume_mm3, exact
    );
    assert!(dev < 1e-9);
    assert_eq!(s.genus, 1, "bore handle survives the pocket");
    assert_eq!(chi(&s), 0);
}

#[test]
fn gate_m5_demo_file_exact_composition_and_stl() {
    // examples/sketch-plate.pcm end-to-end: solved sketch (LM) -> closed
    // regions -> extrude -> cut. Bore sampled at chord tol 0.02 mm on an
    // r=5 circle: 35-gon (verified below), pocket cuts exactly 18*14*4.
    let segs35 = {
        // same sagitta rule as loops::Sample::Chord(0.02), r=5
        let dmax = 2.0 * (1.0f64 - 0.02 / 5.0).acos();
        (TAU / dmax).ceil() as usize
    };
    let src = include_str!("../../../examples/sketch-plate.pcm");
    let m = parse_model(src, "examples/sketch-plate.pcm").unwrap();
    let exact = (2400.0 - ngon_area(5.0, segs35)) * 12.0 - 18.0 * 14.0 * 4.0;
    let s = evaluate_with(&m.graph, &m.sketches).unwrap();
    let dev = (s.volume_mm3 - exact).abs();
    eprintln!(
        "GATE sketch-plate.pcm: V={:.9} exact={:.9} (bore {segs35}-gon) dev {dev:.3e} \
         mass={:.4}g genus={} tris={}",
        s.volume_mm3,
        exact,
        s.mass_g(m.density_g_cm3),
        s.genus,
        s.num_tri
    );
    assert!(dev < 1e-6, "demo composition dev {dev:.3e}");
    assert_eq!(s.genus, 1);
    assert_eq!(chi(&s), 0);

    // STL export round trip + bit-stability of the whole pipeline
    let s2 = evaluate_with(&m.graph, &m.sketches).unwrap();
    let b1 = to_stl_binary(&s.mesh);
    let b2 = to_stl_binary(&s2.mesh);
    assert_eq!(b1, b2, "rebuild must be bit-stable");
    let back = from_stl_binary(&b1).unwrap();
    assert_eq!(back.triangle_count(), s.num_tri);
    // f64-summed volume of the re-read (f32) mesh: STL quantization drift
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
    let rel = (vol64 - s.volume_mm3).abs() / s.volume_mm3;
    eprintln!("GATE sketch-plate STL round trip: rel vol drift {rel:.3e}");
    assert!(rel < 1e-8, "STL quantization rel {rel:.3e}");
}

#[test]
fn gate_two_regions_in_one_sketch_union_in_one_node() {
    // one sketch, two disjoint rectangles -> two regions -> one node
    let second = "q2 = point(s, 30, 0)\nq3 = point(s, 35, 0)\nq4 = point(s, 35, 5)\nq5 = point(s, 30, 5)\n\
         q23 = line(s, q2, q3)\nq34 = line(s, q3, q4)\nq45 = line(s, q4, q5)\nq52 = line(s, q5, q2)\n";
    let src = format!(
        "{}{}\nslab = extrude(s, 2)\n",
        rect_sketch("s", 0.0, 0.0, 5.0, 5.0),
        second
    );
    let s = build(&src, "two-regions.pcm");
    let dev = (s.volume_mm3 - 100.0).abs();
    eprintln!(
        "GATE two-region extrude: got {:.9} exact 100 dev {dev:.3e}",
        s.volume_mm3
    );
    assert!(dev < 1e-9);
    // M3 verified convention: two disjoint shells -> genus -1
    assert_eq!(s.genus, -1);
    assert_eq!(chi(&s), 4, "two sphere-topology shells");
}

#[test]
fn gate_honest_errors() {
    // open sketch
    let m = parse_model(
        "s = newsketch()\na = point(s,0,0)\nb = point(s,1,0)\nc = point(s,1,1)\nl1 = line(s,a,b)\nl2 = line(s,b,c)\nx = extrude(s,1)\n",
        "open.pcm",
    )
    .expect("open sketch parses");
    let err = match evaluate_with(&m.graph, &m.sketches) {
        Ok(_) => panic!("open sketch must not build"),
        Err(k) => k.to_string(),
    };
    assert!(
        err.contains("dangling") || err.contains("must close"),
        "{err}"
    );

    // straddling the revolve axis
    let src = format!(
        "{}ax0 = point(s, 0, -2)\nax1 = point(s, 0, 10)\naxl = line(s, ax0, ax1)\nr = revolve(s, axl)\n",
        rect_sketch("s", -2.0, 0.0, 3.0, 4.0)
    );
    let err = build_err(&src, "straddle.pcm");
    assert!(err.contains("straddles"), "{err}");

    // axis operand must be a line entity
    let err = parse_model(
        "s = newsketch()\np = point(s, 5, 0)\nr = revolve(s, p)\n",
        "axisbad.pcm",
    )
    .expect_err("parse must reject")
    .msg;
    assert!(err.contains("must be a line entity"), "{err}");

    // undefined sketch operand
    let err = parse_model("x = extrude(nope, 1)", "nosk.pcm")
        .expect_err("parse must reject")
        .msg;
    assert!(err.contains("undefined sketch"), "{err}");
}

fn build_err(src: &str, name: &str) -> String {
    let m = parse_model(src, name).expect("parses");
    match evaluate_with(&m.graph, &m.sketches) {
        Ok(_) => panic!("must not build"),
        Err(e) => e.to_string(),
    }
}

#[test]
fn gate_offset_extrude_places_z_span() {
    // extrude(SK, H, OFF): z in [OFF, OFF+H] — centroid check via volume
    // symmetry: box 10x4 extruded 6 offset 3 -> signed volume unchanged
    // (240) and bbox from display mesh pins the placement.
    let src = format!(
        "{}\nslab = extrude(s, 6, 3)\n",
        rect_sketch("s", 0.0, 0.0, 10.0, 4.0)
    );
    let s = build(&src, "offset.pcm");
    assert!((s.volume_mm3 - 240.0).abs() < 1e-9);
    let zs: Vec<f32> = s.mesh.positions.iter().map(|p| p[2]).collect();
    let (zmin, zmax) = (
        zs.iter().cloned().fold(f32::INFINITY, f32::min),
        zs.iter().cloned().fold(f32::NEG_INFINITY, f32::max),
    );
    assert!(
        (zmin - 3.0).abs() < 1e-6 && (zmax - 9.0).abs() < 1e-6,
        "z span {zmin}..{zmax}"
    );
}
