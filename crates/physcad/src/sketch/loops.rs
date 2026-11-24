//! M5: turn a (solved) sketch's curve entities into closed 2D regions.
//!
//! Pipeline: entities -> polyline chains -> weld endpoints -> closed walks
//! -> containment nesting -> `Region2`s (one outer loop + its holes).
//!
//! Welding is by *distance tolerance*, not entity identity: coincident
//! constraints merge point *parameters* in the solver, but two point
//! entities can still exist at the same (solved) coordinates, and LM
//! residuals are ~1e-14, not bit-exact. So endpoints snap to weld nodes.
//!
//! Orientation contract (used by `crate::sweep`): every region's outer
//! loop is CCW and every hole loop is CW — "material stays on the left"
//! while walking any loop. This makes wall/cap winding one uniform rule
//! for extrude and revolve alike (derived via the M2 basis rule).
//!
//! Documented limits (honest errors, no panics):
//! - A vertex where more than two chains meet is rejected (ambiguous
//!   region boundary — proper planar-face extraction is a later milestone).
//! - Self-crossing loops are *not* detected here; the cap triangulator
//!   will usually refuse to bridge/clip them (sweep::SweepError).
//! - Nesting deeper than island-in-hole-in-island is supported via the
//!   even/odd depth rule (every even-depth loop owns a region).

use std::f64::consts::TAU;

use super::{Sketch, SketchError};

/// Arc/circle sampling control for loop extraction.
#[derive(Clone, Copy, Debug)]
pub enum Sample {
    /// Keep chord sagitta below `tol` mm (renderer-style adaptive).
    Chord(f64),
    /// Fixed segment count per full turn — what the exact-polyhedral
    /// volume gates use (the generated polygon is then *the* polyhedron).
    Segs(usize),
}

impl Sample {
    /// Segments for an arc of `sweep` radians at radius `r`.
    pub fn segments(&self, sweep: f64, r: f64) -> usize {
        match *self {
            Sample::Segs(n) => {
                let segs = ((n as f64) * sweep.abs() / TAU).round().max(1.0);
                segs as usize
            }
            Sample::Chord(tol) => {
                // sagitta s = r(1 - cos(d/2)) <= tol
                let dmax = if tol >= r {
                    TAU / 48.0
                } else {
                    2.0 * (1.0 - tol / r).acos()
                };
                (sweep.abs() / dmax).ceil().max(1.0) as usize
            }
        }
    }
}

/// A closed planar region: outer loop CCW, hole loops CW, no duplicated
/// closing vertex. Coordinates are millimetres in the sketch plane.
#[derive(Clone, Debug, PartialEq)]
pub struct Region2 {
    pub outer: Vec<[f64; 2]>,
    pub holes: Vec<Vec<[f64; 2]>>,
}

fn sample_poly(c: [f64; 2], r: f64, a0: f64, sweep: f64, segs: usize) -> Vec<[f64; 2]> {
    let d = sweep / segs as f64;
    (0..=segs)
        .map(|k| {
            let a = a0 + d * k as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect()
}

fn signed_area(pts: &[[f64; 2]]) -> f64 {
    let mut s = 0.0;
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        s += a[0] * b[1] - b[0] * a[1];
    }
    s / 2.0
}

/// Classic even-odd ray cast (horizontal ray, half-open edge rule).
fn point_in_poly(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let t = (p[1] - a[1]) / (b[1] - a[1]);
            if p[0] < a[0] + t * (b[0] - a[0]) {
                inside = !inside;
            }
        }
    }
    inside
}

/// One curve entity flattened to a polyline (open chains; circles closed).
struct Chain {
    ent: String,
    pts: Vec<[f64; 2]>,
    /// true for self-closing rings (circles)
    ring: bool,
    /// weld-node ids of the two ends (endpoints only)
    a: usize,
    b: usize,
}

/// Extract closed regions. `skip` lists entity indices to ignore —
/// revolve passes its axis line (a construction entity that must not
/// join the region boundary).
pub fn regions(
    sk: &Sketch,
    sample: Sample,
    weld_tol: f64,
    skip: &[usize],
) -> Result<Vec<Region2>, SketchError> {
    let pt = |i: usize| match sk.entities[i].1 {
        super::Entity::Point { x, y } => [x, y],
        _ => [f64::NAN; 2], // kind-checked at entity add time
    };

    // 1) entity -> chains
    let mut chains: Vec<Chain> = Vec::new();
    for (ei, (name, e)) in sk.entities.iter().enumerate() {
        if skip.contains(&ei) {
            continue;
        }
        match e {
            super::Entity::Point { .. } => {}
            super::Entity::Line { a, b } => {
                let (pa, pb) = (pt(*a), pt(*b));
                if (pa[0] - pb[0]).hypot(pa[1] - pb[1]) < weld_tol {
                    return Err(SketchError(format!(
                        "sketch {}: line {name:?} has coincident endpoints — zero-length edge",
                        sk.name
                    )));
                }
                chains.push(Chain {
                    ent: name.clone(),
                    pts: vec![pa, pb],
                    ring: false,
                    a: usize::MAX,
                    b: usize::MAX,
                });
            }
            super::Entity::Circle { center, radius } => {
                if *radius <= weld_tol {
                    return Err(SketchError(format!(
                        "sketch {}: circle {name:?} radius {radius} is too small to close a loop",
                        sk.name
                    )));
                }
                let c = pt(*center);
                let n = sample.segments(TAU, *radius);
                let mut pts = sample_poly(c, *radius, 0.0, TAU, n);
                pts.pop(); // TAU lands on 0: drop the duplicated closing vertex
                chains.push(Chain {
                    ent: name.clone(),
                    pts,
                    ring: true,
                    a: usize::MAX,
                    b: usize::MAX,
                });
            }
            super::Entity::Arc { center, start, end } => {
                let (c, s, e2) = (pt(*center), pt(*start), pt(*end));
                let r = ((s[0] - c[0]).powi(2) + (s[1] - c[1]).powi(2)).sqrt();
                if r <= weld_tol {
                    return Err(SketchError(format!(
                        "sketch {}: arc {name:?} has zero radius",
                        sk.name
                    )));
                }
                let a0 = (s[1] - c[1]).atan2(s[0] - c[0]);
                let a1 = (e2[1] - c[1]).atan2(e2[0] - c[0]);
                let mut sweep = a1 - a0;
                while sweep <= 0.0 {
                    sweep += TAU;
                }
                let n = sample.segments(sweep, r);
                chains.push(Chain {
                    ent: name.clone(),
                    pts: sample_poly(c, r, a0, sweep, n),
                    ring: false,
                    a: usize::MAX,
                    b: usize::MAX,
                });
            }
        }
    }
    if chains.is_empty() {
        return Err(SketchError(format!(
            "sketch {}: no curve entities to close a region from",
            sk.name
        )));
    }

    // 2) weld endpoints
    let mut weld: Vec<[f64; 2]> = Vec::new();
    let mut weld_id = |p: [f64; 2]| -> usize {
        if let Some(i) = weld
            .iter()
            .position(|&q| (q[0] - p[0]).hypot(q[1] - p[1]) <= weld_tol)
        {
            i
        } else {
            weld.push(p);
            weld.len() - 1
        }
    };
    // rings keep no weld nodes: traversal treats them as already closed.
    for ch in chains.iter_mut() {
        if ch.ring {
            continue;
        }
        ch.a = weld_id(ch.pts[0]);
        let last = ch.pts.len() - 1;
        ch.b = weld_id(ch.pts[last]);
    }

    // snap chain endpoints onto weld nodes (bit-identical seams)
    for ch in chains.iter_mut() {
        if ch.ring {
            continue;
        }
        let n = ch.pts.len();
        ch.pts[0] = weld[ch.a];
        ch.pts[n - 1] = weld[ch.b];
    }

    // 3) adjacency over weld nodes
    let mut adj: Vec<Vec<(usize, bool)>> = vec![Vec::new(); weld.len()];
    for (ci, ch) in chains.iter().enumerate() {
        if ch.ring {
            continue;
        }
        adj[ch.a].push((ci, true)); // chain start sits here
        adj[ch.b].push((ci, false)); // chain end sits here
    }
    for (wi, list) in adj.iter().enumerate() {
        if list.len() == 1 {
            return Err(SketchError(format!(
                "sketch {}: chain {:?} ends at a dangling endpoint ({:.6}, {:.6}) — loops must close",
                sk.name, chains[list[0].0].ent, weld[wi][0], weld[wi][1]
            )));
        }
        if list.len() > 2 {
            let names: Vec<&str> = list
                .iter()
                .map(|(ci, _)| chains[*ci].ent.as_str())
                .collect();
            return Err(SketchError(format!(
                "sketch {}: {} chains meet at ({:.6}, {:.6}) — ambiguous boundary ({}); split the sketch",
                sk.name,
                list.len(),
                weld[wi][0],
                weld[wi][1],
                names.join(", ")
            )));
        }
    }

    // 4) traverse closed walks
    let mut loops: Vec<Vec<[f64; 2]>> = Vec::new();
    for ch in &chains {
        if ch.ring {
            loops.push(ch.pts.clone());
        }
    }
    let mut seen_chain = vec![false; chains.len()];
    for start in 0..chains.len() {
        if seen_chain[start] || chains[start].ring {
            continue;
        }
        seen_chain[start] = true;
        let mut pts = chains[start].pts.clone();
        let start_node = chains[start].a;
        let mut node = chains[start].b;
        let mut guard = 0usize;
        let max_steps = chains.len() + 2;
        while node != start_node {
            guard += 1;
            if guard > max_steps {
                return Err(SketchError(format!(
                    "sketch {}: chain walk did not close (cycle bug)",
                    sk.name
                )));
            }
            // take the other chain at `node`
            let next = adj[node]
                .iter()
                .find(|(ci, _)| !seen_chain[*ci])
                .map(|(ci, at_start)| (*ci, *at_start))
                .ok_or_else(|| {
                    SketchError(format!(
                        "sketch {}: open chain at ({:.6}, {:.6}) — loops must close",
                        sk.name, weld[node][0], weld[node][1]
                    ))
                })?;
            let (ci, at_start) = next;
            seen_chain[ci] = true;
            let ch = &chains[ci];
            // orient chain so it begins at `node`; append its interior+end
            if at_start {
                pts.extend_from_slice(&ch.pts[1..]);
                node = ch.b;
            } else {
                let rev: Vec<[f64; 2]> = ch.pts.iter().rev().copied().collect();
                pts.extend_from_slice(&rev[1..]);
                node = ch.a;
            }
        }
        // drop the duplicated closing vertex (walk returned to start point)
        if pts.len() > 1 {
            pts.pop();
        }
        if pts.len() < 3 {
            return Err(SketchError(format!(
                "sketch {}: degenerate closed walk with {} points",
                sk.name,
                pts.len()
            )));
        }
        loops.push(pts);
    }
    if loops.is_empty() {
        return Err(SketchError(format!(
            "sketch {}: no closed loops found",
            sk.name
        )));
    }

    // normalize orientation: every loop CCW first (containment/depth)
    for l in &mut loops {
        if signed_area(l) < 0.0 {
            l.reverse();
        }
    }

    // 5) nesting: depth = number of containers; parent = the container
    // with exactly one fewer container (the direct parent in the forest).
    let n = loops.len();
    let mut contains: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        let p = loops[i][0];
        for (j, l) in loops.iter().enumerate() {
            if i != j && point_in_poly(p, l) {
                contains[i].push(j);
            }
        }
    }
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by_key(|i| contains[*i].len()); // fewest containers first
    let mut depth = vec![0usize; n];
    let mut parent = vec![usize::MAX; n];
    for &i in &order {
        depth[i] = contains[i].len();
        // parent = container with exactly one fewer container than i
        let mut found = usize::MAX;
        for &j in &contains[i] {
            if contains[j].len() + 1 == contains[i].len() {
                found = j;
            }
        }
        parent[i] = found;
    }
    for i in 0..n {
        if depth[i] > 0 && parent[i] == usize::MAX {
            return Err(SketchError(format!(
                "sketch {}: loop #{i} is nested but has no direct container (overlapping loops?)",
                sk.name
            )));
        }
    }

    // 6) regions: even-depth loops are outers (CCW), odd-depth are holes (CW)
    let mut out: Vec<Region2> = Vec::new();
    for i in 0..n {
        if !depth[i].is_multiple_of(2) {
            continue;
        }
        let mut holes = Vec::new();
        for j in 0..n {
            if parent[j] == i && !depth[j].is_multiple_of(2) {
                let mut h = loops[j].clone();
                h.reverse(); // holes walk CW (material on the left)
                holes.push(h);
            }
        }
        out.push(Region2 {
            outer: loops[i].clone(),
            holes,
        });
    }
    if out.is_empty() {
        return Err(SketchError(format!(
            "sketch {}: all loops are holes with no outer boundary",
            sk.name
        )));
    }
    // reject zero-area loops
    for r in &out {
        if signed_area(&r.outer).abs() <= weld_tol * weld_tol {
            return Err(SketchError(format!(
                "sketch {}: region outer loop has zero area",
                sk.name
            )));
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::Entity;

    #[test]
    fn rectangle_only_gives_one_region() {
        let mut sk = Sketch::new("s");
        let a = sk
            .add_entity("a", Entity::Point { x: 0.0, y: 0.0 })
            .unwrap();
        let b = sk
            .add_entity("b", Entity::Point { x: 10.0, y: 0.0 })
            .unwrap();
        let c = sk
            .add_entity("c", Entity::Point { x: 10.0, y: 4.0 })
            .unwrap();
        let d = sk
            .add_entity("d", Entity::Point { x: 0.0, y: 4.0 })
            .unwrap();
        sk.add_entity("ab", Entity::Line { a, b }).unwrap();
        sk.add_entity("bc", Entity::Line { a: b, b: c }).unwrap();
        sk.add_entity("cd", Entity::Line { a: c, b: d }).unwrap();
        sk.add_entity("da", Entity::Line { a: d, b: a }).unwrap();
        let regs = regions(&sk, Sample::Segs(64), 1e-9, &[]).unwrap();
        assert_eq!(regs.len(), 1);
        assert_eq!(regs[0].holes.len(), 0);
        assert_eq!(regs[0].outer.len(), 4);
        assert!(signed_area(&regs[0].outer) > 0.0, "outer CCW");
    }

    #[test]
    fn circle_hole_inside_rectangle_is_a_hole_and_walks_cw() {
        let mut sk = Sketch::new("s");
        let a = sk
            .add_entity("a", Entity::Point { x: 0.0, y: 0.0 })
            .unwrap();
        let b = sk
            .add_entity("b", Entity::Point { x: 20.0, y: 0.0 })
            .unwrap();
        let c = sk
            .add_entity("c", Entity::Point { x: 20.0, y: 20.0 })
            .unwrap();
        let d = sk
            .add_entity("d", Entity::Point { x: 0.0, y: 20.0 })
            .unwrap();
        sk.add_entity("ab", Entity::Line { a, b }).unwrap();
        sk.add_entity("bc", Entity::Line { a: b, b: c }).unwrap();
        sk.add_entity("cd", Entity::Line { a: c, b: d }).unwrap();
        sk.add_entity("da", Entity::Line { a: d, b: a }).unwrap();
        let oc = sk
            .add_entity("oc", Entity::Point { x: 10.0, y: 10.0 })
            .unwrap();
        sk.add_entity(
            "h",
            Entity::Circle {
                center: oc,
                radius: 4.0,
            },
        )
        .unwrap();
        let regs = regions(&sk, Sample::Segs(16), 1e-9, &[]).unwrap();
        assert_eq!(regs.len(), 1);
        assert_eq!(regs[0].holes.len(), 1);
        assert_eq!(regs[0].holes[0].len(), 16);
        assert!(signed_area(&regs[0].holes[0]) < 0.0, "hole CW");
    }

    #[test]
    fn two_disjoint_rectangles_give_two_regions() {
        let mut sk = Sketch::new("s");
        // hand-rolled twice with distinct names via helper-free inline code
        for (k, x0) in [(0, 0.0), (1, 30.0)] {
            let a = sk
                .add_entity(&format!("a{k}"), Entity::Point { x: x0, y: 0.0 })
                .unwrap();
            let b = sk
                .add_entity(
                    &format!("b{k}"),
                    Entity::Point {
                        x: x0 + 5.0,
                        y: 0.0,
                    },
                )
                .unwrap();
            let c = sk
                .add_entity(
                    &format!("c{k}"),
                    Entity::Point {
                        x: x0 + 5.0,
                        y: 5.0,
                    },
                )
                .unwrap();
            let d = sk
                .add_entity(&format!("d{k}"), Entity::Point { x: x0, y: 5.0 })
                .unwrap();
            sk.add_entity(&format!("ab{k}"), Entity::Line { a, b })
                .unwrap();
            sk.add_entity(&format!("bc{k}"), Entity::Line { a: b, b: c })
                .unwrap();
            sk.add_entity(&format!("cd{k}"), Entity::Line { a: c, b: d })
                .unwrap();
            sk.add_entity(&format!("da{k}"), Entity::Line { a: d, b: a })
                .unwrap();
        }
        let regs = regions(&sk, Sample::Segs(64), 1e-9, &[]).unwrap();
        assert_eq!(regs.len(), 2);
    }

    #[test]
    fn island_in_hole_creates_a_second_region() {
        let mut sk = Sketch::new("s");
        rect_outer(&mut sk);
        let h1 = [[5.0, 5.0], [15.0, 5.0], [15.0, 15.0], [5.0, 15.0]];
        add_poly(&mut sk, "h", &h1, true);
        let i1 = [[8.0, 8.0], [12.0, 8.0], [12.0, 12.0], [8.0, 12.0]];
        add_poly(&mut sk, "is", &i1, false);
        let regs = regions(&sk, Sample::Segs(64), 1e-9, &[]).unwrap();
        assert_eq!(regs.len(), 2, "outer-with-hole + island");
        assert_eq!(regs[0].holes.len(), 1);
        assert_eq!(regs[1].holes.len(), 0);
        assert_eq!(regs[1].outer.len(), 4);
    }

    fn rect_outer(sk: &mut Sketch) {
        add_poly(
            sk,
            "o",
            &[[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0]],
            false,
        );
    }

    /// Add a closed polygon with named prefix; reversed=true walks CW.
    fn add_poly(sk: &mut Sketch, prefix: &str, pts: &[[f64; 2]], reverse: bool) {
        let mut ps: Vec<[f64; 2]> = pts.to_vec();
        if reverse {
            ps.reverse();
        }
        let n = ps.len();
        let mut ids = Vec::new();
        for (i, p) in ps.iter().enumerate() {
            ids.push(
                sk.add_entity(&format!("{prefix}{i}"), Entity::Point { x: p[0], y: p[1] })
                    .unwrap(),
            );
        }
        for i in 0..n {
            sk.add_entity(
                &format!("{prefix}l{i}"),
                Entity::Line {
                    a: ids[i],
                    b: ids[(i + 1) % n],
                },
            )
            .unwrap();
        }
    }

    #[test]
    fn open_chain_is_rejected_with_a_located_message() {
        let mut sk = Sketch::new("s");
        let a = sk
            .add_entity("a", Entity::Point { x: 0.0, y: 0.0 })
            .unwrap();
        let b = sk
            .add_entity("b", Entity::Point { x: 5.0, y: 0.0 })
            .unwrap();
        sk.add_entity("solo", Entity::Line { a, b }).unwrap();
        let e = regions(&sk, Sample::Segs(64), 1e-9, &[]).unwrap_err();
        assert!(e.0.contains("dangling"), "{e}");
        assert!(e.0.contains("solo"), "{e}");
    }

    #[test]
    fn vertex_junction_is_rejected() {
        let mut sk = Sketch::new("s");
        // two squares sharing exactly the corner (0,0): degree 4 there
        add_poly(
            &mut sk,
            "q",
            &[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
            false,
        );
        add_poly(
            &mut sk,
            "r",
            &[[0.0, 0.0], [-10.0, 0.0], [-10.0, -10.0], [0.0, -10.0]],
            false,
        );
        let e = regions(&sk, Sample::Segs(64), 1e-9, &[]).unwrap_err();
        assert!(e.0.contains("meet at"), "{e}");
    }

    #[test]
    fn coincident_via_two_point_entities_welds() {
        // two separate point entities at the same solved coords
        let mut sk = Sketch::new("s");
        let a = sk
            .add_entity("a", Entity::Point { x: 0.0, y: 0.0 })
            .unwrap();
        let b = sk
            .add_entity("b", Entity::Point { x: 10.0, y: 0.0 })
            .unwrap();
        let b2 = sk
            .add_entity("b2", Entity::Point { x: 10.0, y: 1e-14 })
            .unwrap();
        let c = sk
            .add_entity("c", Entity::Point { x: 10.0, y: 4.0 })
            .unwrap();
        let d = sk
            .add_entity("d", Entity::Point { x: 0.0, y: 4.0 })
            .unwrap();
        let a2 = sk
            .add_entity("a2", Entity::Point { x: 1e-14, y: 0.0 })
            .unwrap();
        sk.add_entity("ab", Entity::Line { a, b }).unwrap();
        sk.add_entity("bc", Entity::Line { a: b2, b: c }).unwrap();
        sk.add_entity("cd", Entity::Line { a: c, b: d }).unwrap();
        sk.add_entity("da", Entity::Line { a: d, b: a2 }).unwrap();
        let regs = regions(&sk, Sample::Segs(64), 1e-9, &[]).unwrap();
        assert_eq!(regs[0].outer.len(), 4);
    }
}
