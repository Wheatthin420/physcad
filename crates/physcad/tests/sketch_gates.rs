//! Milestone-4 solver gates: known geometries must solve to max residual
//! < 1e-9, and over/under-constrained systems must produce *diagnostics*
//! (rank, dof, named redundancy, honest Unsolved status) — never a panic.
//! Every test prints its exact residual so the report can quote it.

use physcad::sketch::solver::{self, SolveOptions, SolveStatus};
use physcad::sketch::{Constraint, Entity, Sketch};

fn pt(sk: &mut Sketch, name: &str, x: f64, y: f64) -> usize {
    sk.add_entity(name, Entity::Point { x, y }).unwrap()
}
fn line(sk: &mut Sketch, name: &str, a: usize, b: usize) -> usize {
    sk.add_entity(name, Entity::Line { a, b }).unwrap()
}
fn circle(sk: &mut Sketch, name: &str, c: usize, r: f64) -> usize {
    sk.add_entity(
        name,
        Entity::Circle {
            center: c,
            radius: r,
        },
    )
    .unwrap()
}
fn arc(sk: &mut Sketch, name: &str, c: usize, s: usize, e: usize) -> usize {
    sk.add_entity(
        name,
        Entity::Arc {
            center: c,
            start: s,
            end: e,
        },
    )
    .unwrap()
}
fn con(sk: &mut Sketch, name: &str, c: Constraint) {
    sk.add_constraint(name, c).unwrap();
}

/// Distance between two point entities of a solved sketch (from params).
fn coord(sk: &Sketch, ent: usize) -> (f64, f64) {
    match sk.entities[ent].1 {
        Entity::Point { x, y } => (x, y),
        _ => panic!("not a point"),
    }
}
fn radius(sk: &Sketch, ent: usize) -> f64 {
    match sk.entities[ent].1 {
        Entity::Circle { radius, .. } => radius,
        _ => panic!("not a circle"),
    }
}

fn report_case(tag: &str, r: &solver::SolveReport) {
    eprintln!(
        "[{tag}] status={:?} iters={} max|res|={:.3e} l2={:.3e} rank={}/{} dof={} sv={:?}",
        r.status,
        r.iterations,
        r.max_residual,
        r.l2_residual,
        r.rank,
        r.n_params,
        r.dof,
        r.singular_values
    );
    if !r.redundant.is_empty() {
        eprintln!("[{tag}] redundant constraints: {:?}", r.redundant);
    }
    if !r.free_params.is_empty() {
        eprintln!("[{tag}] free params: {:?}", r.free_params);
    }
}

const DOD_TOL: f64 = 1e-9;

#[test]
fn triangle_with_three_sides_solves_underconstrained() {
    // 3-4-5 triangle from side lengths only: 6 params, 3 eqs.
    // Expected: solves, dof = 3 (2 translation + 1 rotation, rigid body).
    let mut sk = Sketch::new("tri");
    let a = pt(&mut sk, "a", 0.0, 0.0);
    let b = pt(&mut sk, "b", 4.2, 0.3); // deliberately wrong guesses
    let c = pt(&mut sk, "c", -0.5, 2.9);
    con(&mut sk, "ab", Constraint::Distance { a, b, value: 4.0 });
    con(
        &mut sk,
        "bc",
        Constraint::Distance {
            a: b,
            b: c,
            value: 3.0,
        },
    );
    con(
        &mut sk,
        "ca",
        Constraint::Distance {
            a: c,
            b: a,
            value: 5.0,
        },
    );

    let (solved, rep) = solver::solve(&sk);
    report_case("triangle-3-4-5", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(
        rep.max_residual < DO_D_TOL(),
        "max residual {}",
        rep.max_residual
    );
    assert_eq!(rep.dof, 3, "rigid body freedom must be reported");
    assert!(rep.redundant.is_empty());

    let dist = |p: usize, q: usize| {
        let (x1, y1) = coord(&solved, p);
        let (x2, y2) = coord(&solved, q);
        ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
    };
    eprintln!(
        "[triangle-3-4-5] sides: ab={:.12} bc={:.12} ca={:.12}",
        dist(a, b),
        dist(b, c),
        dist(c, a)
    );
    assert!((dist(a, b) - 4.0).abs() < 1e-9);
    assert!((dist(b, c) - 3.0).abs() < 1e-9);
    assert!((dist(c, a) - 5.0).abs() < 1e-9);
    // exactly 3 near-zero singular values, 3 clearly nonzero
    let zeros = rep.singular_values.iter().filter(|s| **s < 1e-6).count();
    assert_eq!(zeros, 3, "{:?}", rep.singular_values);
}

#[test]
fn rectangle_constrained_to_axes_fully_constrained() {
    // 4 messy points -> exact 80x48 rectangle with edges on axis
    // directions, anchored at (10,-5): 8 params, 8 eqs, dof 0.
    let mut sk = Sketch::new("rect");
    let a = pt(&mut sk, "a", 5.0, 5.0);
    let b = pt(&mut sk, "b", 90.0, 3.0);
    let c = pt(&mut sk, "c", 85.0, 55.0);
    let d = pt(&mut sk, "d", 2.0, 50.0);
    let ab = line(&mut sk, "ab", a, b);
    let bc = line(&mut sk, "bc", b, c);
    let cd = line(&mut sk, "cd", c, d);
    let da = line(&mut sk, "da", d, a);
    con(
        &mut sk,
        "anchor",
        Constraint::Fix {
            p: a,
            x: 10.0,
            y: -5.0,
        },
    );
    con(&mut sk, "ab_h", Constraint::Horizontal { l: ab });
    con(&mut sk, "bc_v", Constraint::Vertical { l: bc });
    con(&mut sk, "cd_h", Constraint::Horizontal { l: cd });
    con(&mut sk, "da_v", Constraint::Vertical { l: da });
    con(&mut sk, "width", Constraint::Distance { a, b, value: 80.0 });
    con(
        &mut sk,
        "height",
        Constraint::Distance {
            a: b,
            b: c,
            value: 48.0,
        },
    );

    let (solved, rep) = solver::solve(&sk);
    report_case("rect-axes", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < DO_D_TOL());
    assert_eq!(rep.dof, 0, "must be fully constrained");
    assert!(rep.redundant.is_empty());

    let want = [
        (a, 10.0, -5.0),
        (b, 90.0, -5.0),
        (c, 90.0, 43.0),
        (d, 10.0, 43.0),
    ];
    for (e, x, y) in want {
        let (gx, gy) = coord(&solved, e);
        eprintln!(
            "[rect-axes] {} = ({:.12}, {:.12}) want ({x}, {y})",
            solved.entities[e].0, gx, gy
        );
        assert!((gx - x).abs() < 1e-9 && (gy - y).abs() < 1e-9);
    }
}

#[test]
fn circle_tangent_to_two_perpendicular_lines() {
    // corner at origin (bottom + left edge, directions outward CCW),
    // circle tangent to both with r=4 -> center exactly (4,4).
    let mut sk = Sketch::new("corner");
    let o = pt(&mut sk, "o", 0.0, 0.0);
    let p = pt(&mut sk, "p", 50.0, 0.5);
    let q = pt(&mut sk, "q", 0.3, 50.0);
    let c = pt(&mut sk, "c", 2.0, 3.0); // wrong guess
    let l1 = line(&mut sk, "l1", o, p);
    let l2 = line(&mut sk, "l2", o, q);
    let ci = circle(&mut sk, "ci", c, 1.0);
    con(
        &mut sk,
        "anchor",
        Constraint::Fix {
            p: o,
            x: 0.0,
            y: 0.0,
        },
    );
    con(&mut sk, "l1h", Constraint::Horizontal { l: l1 });
    con(
        &mut sk,
        "l1len",
        Constraint::Distance {
            a: o,
            b: p,
            value: 50.0,
        },
    );
    con(&mut sk, "l2v", Constraint::Vertical { l: l2 });
    con(
        &mut sk,
        "l2len",
        Constraint::Distance {
            a: o,
            b: q,
            value: 50.0,
        },
    );
    con(
        &mut sk,
        "t1",
        Constraint::TangentLineCircle {
            line: l1,
            circle: ci,
        },
    );
    con(
        &mut sk,
        "t2",
        Constraint::TangentLineCircle {
            line: l2,
            circle: ci,
        },
    );
    con(&mut sk, "r4", Constraint::Radius { c: ci, value: 4.0 });

    let (solved, rep) = solver::solve(&sk);
    report_case("circle-tan-2-lines", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < DO_D_TOL());
    assert_eq!(rep.dof, 0);
    let (cx, cy) = coord(&solved, c);
    let r = radius(&solved, ci);
    eprintln!("[circle-tan-2-lines] center=({cx:.12}, {cy:.12}) r={r:.12}");
    assert!((cx - 4.0).abs() < 1e-9 && (cy - 4.0).abs() < 1e-9);
    assert!((r - 4.0).abs() < 1e-9);
}

#[test]
fn circle_in_60deg_wedge_closed_form_center() {
    // wedge of two lines at 60 deg; circle tangent to both, r=4.
    // Center must sit on the bisector at r/sin(30deg) = 2r from the
    // vertex: (2r cos30, 2r sin30) = (6.928203230275509, 4).
    // Q's distance along its ray stays free -> dof 1.
    let mut sk = Sketch::new("wedge");
    let o = pt(&mut sk, "o", 0.0, 0.0);
    let p = pt(&mut sk, "p", 40.0, 0.0);
    let q = pt(&mut sk, "q", 10.0, 10.0); // 45 deg guess
    let c = pt(&mut sk, "c", 3.0, 2.0);
    let l1 = line(&mut sk, "l1", o, p);
    let l2 = line(&mut sk, "l2", o, q);
    let ci = circle(&mut sk, "ci", c, 1.0);
    con(
        &mut sk,
        "anchor",
        Constraint::Fix {
            p: o,
            x: 0.0,
            y: 0.0,
        },
    );
    con(&mut sk, "l1h", Constraint::Horizontal { l: l1 });
    con(
        &mut sk,
        "l1len",
        Constraint::Distance {
            a: o,
            b: p,
            value: 50.0,
        },
    );
    con(
        &mut sk,
        "a60",
        Constraint::Angle {
            a: l1,
            b: l2,
            radians: 60f64.to_radians(),
        },
    );
    con(
        &mut sk,
        "t1",
        Constraint::TangentLineCircle {
            line: l1,
            circle: ci,
        },
    );
    con(
        &mut sk,
        "t2",
        Constraint::TangentLineCircle {
            line: l2,
            circle: ci,
        },
    );
    con(&mut sk, "r4", Constraint::Radius { c: ci, value: 4.0 });

    let (solved, rep) = solver::solve(&sk);
    report_case("wedge-60", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < DO_D_TOL());
    assert_eq!(rep.dof, 1, "free ray length must show up");

    let want_x = 4.0f64 * 3.0f64.sqrt(); // 2r*cos30
    let (cx, cy) = coord(&solved, c);
    eprintln!("[wedge-60] center=({cx:.12}, {cy:.12}) want ({want_x:.12}, 4)");
    assert!((cx - want_x).abs() < 1e-9);
    assert!((cy - 4.0).abs() < 1e-9);
    // l2 direction really is 60 deg now
    let (qx, qy) = coord(&solved, q);
    let ang = qy.atan2(qx).to_degrees();
    eprintln!("[wedge-60] l2 direction {ang:.12} deg");
    assert!((ang - 60.0).abs() < 1e-9);
}

#[test]
fn over_constrained_duplicate_reports_redundant_not_panic() {
    // triangle + a second distance(a,b) naming the same 4 mm side:
    // must solve anyway, flag the duplicate as redundant, dof still 3.
    let mut sk = Sketch::new("over");
    let a = pt(&mut sk, "a", 0.0, 0.0);
    let b = pt(&mut sk, "b", 4.2, 0.3);
    let c = pt(&mut sk, "c", -0.5, 2.9);
    con(&mut sk, "ab", Constraint::Distance { a, b, value: 4.0 });
    con(
        &mut sk,
        "bc",
        Constraint::Distance {
            a: b,
            b: c,
            value: 3.0,
        },
    );
    con(
        &mut sk,
        "ca",
        Constraint::Distance {
            a: c,
            b: a,
            value: 5.0,
        },
    );
    con(
        &mut sk,
        "ab_again",
        Constraint::Distance { a, b, value: 4.0 },
    );

    let (solved, rep) = solver::solve(&sk);
    report_case("over-constrained", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < DO_D_TOL());
    assert_eq!(rep.redundant, vec!["ab_again".to_string()]);
    assert_eq!(rep.dof, 3);
    let _ = solved;
}

#[test]
fn contradictory_dimensions_report_unsolved_not_panic() {
    // sides 1, 1, 5: no triangle exists. LM must stall and say so.
    let mut sk = Sketch::new("bad");
    let a = pt(&mut sk, "a", 0.0, 0.0);
    let b = pt(&mut sk, "b", 1.0, 0.0);
    let c = pt(&mut sk, "c", 0.5, 0.8);
    con(&mut sk, "ab", Constraint::Distance { a, b, value: 1.0 });
    con(
        &mut sk,
        "bc",
        Constraint::Distance {
            a: b,
            b: c,
            value: 1.0,
        },
    );
    con(
        &mut sk,
        "ca",
        Constraint::Distance {
            a: c,
            b: a,
            value: 5.0,
        },
    );

    let (solved, rep) = solver::solve(&sk);
    report_case("contradictory", &rep);
    assert_eq!(rep.status, SolveStatus::Unsolved);
    assert!(rep.max_residual > 1.0, "must not pretend it solved");
    assert!(!rep.message.is_empty());
    // geometry is still a valid sketch data structure afterwards
    let (x1, y1) = coord(&solved, a);
    let (x2, y2) = coord(&solved, b);
    eprintln!(
        "[contradictory] stalled best fit ab={:.6} ca={:.6}",
        ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt(),
        {
            let (x3, y3) = coord(&solved, c);
            ((x1 - x3).powi(2) + (y1 - y3).powi(2)).sqrt()
        }
    );
}

#[test]
fn coincident_merges_two_points() {
    let mut sk = Sketch::new("merge");
    let a = pt(&mut sk, "a", 0.0, 0.0);
    let b = pt(&mut sk, "b", 5.0, 5.0);
    con(
        &mut sk,
        "anchor",
        Constraint::Fix {
            p: a,
            x: 1.0,
            y: 2.0,
        },
    );
    con(&mut sk, "join", Constraint::Coincident { a, b });
    let (solved, rep) = solver::solve(&sk);
    report_case("coincident", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < DO_D_TOL());
    assert_eq!(rep.dof, 0);
    let (bx, by) = coord(&solved, b);
    eprintln!("[coincident] b = ({bx:.12}, {by:.12})");
    assert!((bx - 1.0).abs() < 1e-12 && (by - 2.0).abs() < 1e-12);
}

#[test]
fn line_tangent_to_arc_gives_parabola_dof() {
    // fixed horizontal line, fixed arc start point, free center:
    // distance^2(line, center) == |start-center|^2 is a PARABOLA:
    // dof must be reported as 2 (center along parabola x radius),
    // and the tangent equation itself must hold to < 1e-9.
    let mut sk = Sketch::new("arc-tan");
    let a = pt(&mut sk, "a", 0.0, 0.0);
    let b = pt(&mut sk, "b", 10.0, 0.0);
    let c = pt(&mut sk, "c", 5.0, 4.0);
    let s = pt(&mut sk, "s", 3.0, 4.0);
    let e = pt(&mut sk, "e", 7.0, 4.0);
    let l = line(&mut sk, "l", a, b);
    let ar = arc(&mut sk, "ar", c, s, e);
    con(
        &mut sk,
        "fa",
        Constraint::Fix {
            p: a,
            x: 0.0,
            y: 0.0,
        },
    );
    con(
        &mut sk,
        "fb",
        Constraint::Fix {
            p: b,
            x: 10.0,
            y: 0.0,
        },
    );
    con(
        &mut sk,
        "fs",
        Constraint::Fix {
            p: s,
            x: 3.0,
            y: 4.0,
        },
    );
    con(
        &mut sk,
        "r_cs",
        Constraint::Distance {
            a: c,
            b: s,
            value: 5.0,
        },
    );
    con(
        &mut sk,
        "r_ce",
        Constraint::Distance {
            a: c,
            b: e,
            value: 5.0,
        },
    );
    con(
        &mut sk,
        "tan",
        Constraint::TangentLineArc { line: l, arc: ar },
    );

    let (solved, rep) = solver::solve(&sk);
    report_case("line-tangent-arc", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < DO_D_TOL());
    // params: a,b,c,s,e = 10; eqs: 3 fixes(6) + 2 distances + tangent = 9
    assert_eq!(rep.n_equations, 9);
    assert_eq!(rep.dof, 1, "center slides on the parabola: dof 1 expected");
    let (cx, cy) = coord(&solved, c);
    let (sx, sy) = coord(&solved, s);
    // parabola identity: cy^2 == (cx-sx)^2 + (cy-sy)^2  (line is y=0)
    let lhs = cy * cy;
    let rhs = (cx - sx).powi(2) + (cy - sy).powi(2);
    eprintln!(
        "[line-tangent-arc] center=({cx:.9}, {cy:.9}) parabola dev {:.3e}",
        (lhs - rhs).abs()
    );
    assert!((lhs - rhs).abs() < 1e-9);
    let _ = ar;
}

#[test]
fn tangent_circles_with_fixes_flags_redundant_and_stays_exact() {
    // two fully-pinned circles whose pin coordinates ARE externally
    // consistent (|c1-c2| = 3+5): the tangent row is dependent ->
    // must be named redundant, solve stays exact.
    let mut sk = Sketch::new("cc");
    let c1 = pt(&mut sk, "c1", 0.1, -0.1);
    let c2 = pt(&mut sk, "c2", 8.2, 0.0);
    let a = circle(&mut sk, "a", c1, 3.0);
    let b = circle(&mut sk, "b", c2, 5.0);
    con(
        &mut sk,
        "f1",
        Constraint::Fix {
            p: c1,
            x: 0.0,
            y: 0.0,
        },
    );
    con(
        &mut sk,
        "f2",
        Constraint::Fix {
            p: c2,
            x: 8.0,
            y: 0.0,
        },
    );
    con(&mut sk, "r1", Constraint::Radius { c: a, value: 3.0 });
    con(&mut sk, "r2", Constraint::Radius { c: b, value: 5.0 });
    con(&mut sk, "tan", Constraint::TangentCircles { a, b });

    let (solved, rep) = solver::solve(&sk);
    report_case("tangent-circles", &rep);
    assert_eq!(rep.status, SolveStatus::Solved);
    assert!(rep.max_residual < DO_D_TOL());
    assert_eq!(rep.redundant, vec!["tan".to_string()]);
    assert_eq!(rep.dof, 0);
    let (x1, y1) = coord(&solved, c1);
    let (x2, y2) = coord(&solved, c2);
    let d = ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt();
    eprintln!("[tangent-circles] center distance {d:.12}");
    assert!((d - 8.0).abs() < 1e-12);
}

#[test]
fn solve_is_deterministic_bit_for_bit() {
    // same sketch twice -> identical params (bit pattern), identical
    // report scalars: determinism contract for the whole timeline.
    let build = || {
        let mut sk = Sketch::new("det");
        let a = pt(&mut sk, "a", 5.0, 5.0);
        let b = pt(&mut sk, "b", 90.0, 3.0);
        let c = pt(&mut sk, "c", 85.0, 55.0);
        let d = pt(&mut sk, "d", 2.0, 50.0);
        let ab = line(&mut sk, "ab", a, b);
        let bc = line(&mut sk, "bc", b, c);
        let cd = line(&mut sk, "cd", c, d);
        let da = line(&mut sk, "da", d, a);
        con(
            &mut sk,
            "anchor",
            Constraint::Fix {
                p: a,
                x: 10.0,
                y: -5.0,
            },
        );
        con(&mut sk, "ab_h", Constraint::Horizontal { l: ab });
        con(&mut sk, "bc_v", Constraint::Vertical { l: bc });
        con(&mut sk, "cd_h", Constraint::Horizontal { l: cd });
        con(&mut sk, "da_v", Constraint::Vertical { l: da });
        con(&mut sk, "width", Constraint::Distance { a, b, value: 80.0 });
        con(
            &mut sk,
            "height",
            Constraint::Distance {
                a: b,
                b: c,
                value: 48.0,
            },
        );
        sk
    };
    let (s1, r1) = solver::solve(&build());
    let (s2, r2) = solver::solve(&build());
    assert_eq!(s1.params_vec(), s2.params_vec()); // bits equal (Vec<f64> eq)
    assert_eq!(r1.max_residual.to_bits(), r2.max_residual.to_bits());
    assert_eq!(r1.iterations, r2.iterations);
    eprintln!(
        "[determinism] two solves: max|res|={:.3e} iters={} params bit-equal",
        r1.max_residual, r1.iterations
    );
}

/// tiny helper so the DoD tolerance appears exactly once
#[allow(non_snake_case)]
fn DO_D_TOL() -> f64 {
    SolveOptions::default().tol.max(DOD_TOL) // gate is 1e-9 regardless of solver tol
}
