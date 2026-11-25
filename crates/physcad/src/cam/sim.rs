//! M7 cut simulator: G-code TEXT -> modal state -> swept flat-end-mill
//! solids -> manifold booleans against the stock.
//!
//! The HARD COMPAT RULE lives here: the simulator does not read the
//! planner's typed motions; it re-parses the emitted text with the
//! third-party `gcode` crate (MIT OR Apache-2.0, docs/CAM.md) and
//! replays only what the parser reports. A program that would confuse
//! a real GRBL controller (unsupported modal words, arcs, inch mode)
//! fails here loudly instead of cutting silently wrong.
//!
//! Cross-check (mission DoD): removed volume (stock ∩ swept tools) is
//! compared against an INDEPENDENT kernel boolean — stock cut by the
//! pocket prism extruded from the sketch region. Gouge = removal
//! outside the prism; uncut = prism volume the passes never reached.

use manifold3d::Manifold;

use super::CamError;
use crate::csg::{self, Solid};
use crate::sketch::loops::Region2;
use crate::sweep::extrude_region;

#[derive(Clone, Copy, Debug)]
pub struct SimTool {
    /// flat end mill radius, mm
    pub radius: f64,
    /// effective cutting length above the tip, mm
    pub length: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Seg3 {
    pub from: [f64; 3],
    pub to: [f64; 3],
    pub cut: bool,
    pub feed: f64,
}

#[derive(Clone, Debug)]
pub struct SimReport {
    pub segments: Vec<Seg3>,
    pub rapids: usize,
    pub cut_xy: usize,
    pub cut_z: usize,
    pub stock_mm3: f64,
    /// stock ∩ (CSG pocket prism): ground truth removal
    pub pocket_mm3: f64,
    /// stock ∩ (swept tools): what the simulated machine removed
    pub removed_mm3: f64,
    pub inside_mm3: f64,
    /// removed outside the pocket prism (gouge)
    pub gouge_mm3: f64,
    /// pocket volume never reached by a pass
    pub uncut_mm3: f64,
}

/// Codes the replay understands = exactly the dialect subset emitted by
/// `cam::gcode` (docs/CAM.md). Anything else is a loud error.
fn general_allowed(g: u32) -> bool {
    matches!(g, 0 | 1 | 4 | 17 | 21 | 40 | 90 | 94)
}

fn prismatic(parts: &mut Vec<Manifold>, m: Manifold) {
    parts.push(m);
}

fn union_all(mut parts: Vec<Manifold>) -> Result<Manifold, CamError> {
    if parts.is_empty() {
        // empty operand: kernel-empty manifold (volume 0), no geometry
        return Manifold::from_mesh_f64(&[], 3, &[])
            .map_err(|e| CamError(format!("empty manifold: {e}")));
    }
    // balanced fold keeps the boolean tree O(log n) deep
    while parts.len() > 1 {
        let cur = std::mem::take(&mut parts);
        let mut next = Vec::with_capacity(cur.len().div_ceil(2));
        let mut it = cur.into_iter();
        while let Some(a) = it.next() {
            match it.next() {
                Some(b) => next.push(a.union(&b)),
                None => next.push(a),
            }
        }
        parts = next;
    }
    Ok(parts.pop().unwrap())
}

/// CCW stadium polygon (segment a->b thickened by r): two straight
/// flanks at distance r plus semicircular caps sampled at n steps.
pub fn stadium(a: [f64; 2], b: [f64; 2], r: f64, n: usize) -> Vec<[f64; 2]> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l = (dx * dx + dy * dy).sqrt();
    if l < 1e-9 {
        // degenerate: circle
        return (0..n)
            .map(|k| {
                let t = std::f64::consts::TAU * k as f64 / n as f64;
                [a[0] + r * t.cos(), a[1] + r * t.sin()]
            })
            .collect();
    }
    let (ux, uy) = (dx / l, dy / l);
    let (nx, ny) = (-uy, ux); // left normal
    let theta = uy.atan2(ux); // direction of travel
    // strict-interior arc points only: the four flanking corners are
    // pushed exactly, so trig endpoints (cos 90deg != 0) never create
    // near-duplicate vertices that would break the manifold audit
    let arc_interior = |c: [f64; 2], from: f64, to: f64| -> Vec<[f64; 2]> {
        let steps = (n as f64 * (to - from) / std::f64::consts::PI)
            .ceil()
            .max(2.0) as usize;
        (1..steps)
            .map(|k| {
                let t = from + (to - from) * k as f64 / steps as f64;
                [c[0] + r * t.cos(), c[1] + r * t.sin()]
            })
            .collect()
    };
    let mut pts = vec![[a[0] - nx * r, a[1] - ny * r]];
    pts.push([b[0] - nx * r, b[1] - ny * r]);
    pts.extend(arc_interior(
        b,
        theta - std::f64::consts::FRAC_PI_2,
        theta + std::f64::consts::FRAC_PI_2,
    ));
    pts.push([b[0] + nx * r, b[1] + ny * r]);
    pts.push([a[0] + nx * r, a[1] + ny * r]);
    pts.extend(arc_interior(
        a,
        theta + std::f64::consts::FRAC_PI_2,
        theta + 3.0 * std::f64::consts::FRAC_PI_2,
    ));
    pts
}

fn prism_from_polygon(poly: &[[f64; 2]], z0: f64, z1: f64) -> Result<Manifold, CamError> {
    let region = Region2 {
        outer: poly.to_vec(),
        holes: vec![],
    };
    let m64 =
        extrude_region(&region, z0, z1).map_err(|e| CamError(format!("sim tool prism: {e}")))?;
    if let Err(e) = m64.closed_manifold_check() {
        return Err(CamError(format!(
            "sim tool prism failed manifold audit: {e}"
        )));
    }
    csg::manifold_from_mesh64(&m64).map_err(|e| CamError(format!("sim kernel: {e}")))
}

/// Parse + replay + measure. `pocket` is the intended pocket footprint
/// (sketch region); the ground-truth prism spans [z_floor, z_top].
pub fn replay(
    text: &str,
    tool: &SimTool,
    stock: &Solid,
    pocket: &Region2,
    z_top: f64,
    z_floor: f64,
) -> Result<SimReport, CamError> {
    let program = gcode::parse(text)
        .map_err(|d| CamError(format!("independent parser rejected program: {d:?}")))?;

    // ---- modal replay ------------------------------------------------------
    let mut pos: Option<[f64; 3]> = None;
    let mut motion = 0u32; // GRBL power-on motion mode default is G0 (wiki $G table)
    let mut feed = 0.0f64;
    let mut segments: Vec<Seg3> = Vec::new();
    for (bi, block) in program.blocks.iter().enumerate() {
        let mut words: Vec<(char, f64)> = Vec::new();
        for code in &block.codes {
            match code {
                gcode::Code::General(g) => {
                    if g.number.minor().is_some() {
                        return Err(CamError(format!(
                            "block {}: G{}.x modal decimal codes not in subset",
                            bi + 1,
                            g.number.major()
                        )));
                    }
                    let gm = g.number.major();
                    match gm {
                        0..=3 => {
                            if gm >= 2 {
                                return Err(CamError(format!(
                                    "block {}: G{gm} arcs not in M7 subset",
                                    bi + 1
                                )));
                            }
                            motion = gm;
                        }
                        20 => {
                            return Err(CamError(format!(
                                "block {}: G20 inch mode not in subset (we emit G21)",
                                bi + 1
                            )));
                        }
                        91 => {
                            return Err(CamError(format!(
                                "block {}: G91 incremental not in subset (we emit G90)",
                                bi + 1
                            )));
                        }
                        93 => {
                            return Err(CamError(format!(
                                "block {}: G93 inverse-time feed not in subset",
                                bi + 1
                            )));
                        }
                        18 | 19 => {
                            return Err(CamError(format!(
                                "block {}: G{gm} plane not in subset (G17 only)",
                                bi + 1
                            )));
                        }
                        41..=43 | 49 | 53 | 80 | 92 => {
                            return Err(CamError(format!(
                                "block {}: G{gm} not in subset (compensation/probe/offset)",
                                bi + 1
                            )));
                        }
                        _ => {
                            if !general_allowed(gm) {
                                return Err(CamError(format!(
                                    "block {}: G{gm} not in subset",
                                    bi + 1
                                )));
                            }
                        }
                    }
                    for a in &g.args {
                        if let gcode::Value::Literal(v) = a.value {
                            words.push((a.letter.to_ascii_uppercase(), f64::from(v)));
                        }
                    }
                }
                gcode::Code::Miscellaneous(mm) => {
                    let mc = mm.number.major();
                    if !matches!(mc, 2..=5) {
                        return Err(CamError(format!(
                            "block {}: M{mc} not in subset (2 3 5 + dwell via G4)",
                            bi + 1
                        )));
                    }
                    for a in &mm.args {
                        if let gcode::Value::Literal(v) = a.value {
                            words.push((a.letter.to_ascii_uppercase(), f64::from(v)));
                        }
                    }
                }
                gcode::Code::ToolChange(t) => {
                    return Err(CamError(format!(
                        "block {}: T{} — M7 has one tool, no tool changes",
                        bi + 1,
                        t.number.major()
                    )));
                }
                // Code is #[non_exhaustive]: any future variant is outside
                // the subset until reviewed, so reject loudly.
                _ => {
                    return Err(CamError(format!(
                        "block {}: code kind not in subset",
                        bi + 1
                    )));
                }
            }
        }
        for w in &block.word_addresses {
            if let gcode::Value::Literal(v) = w.value {
                words.push((w.letter.to_ascii_uppercase(), f64::from(v)));
            }
        }
        if let Some(v) = words.iter().find(|q| q.0 == 'F').map(|q| q.1) {
            feed = v;
        }
        let nx = words.iter().find(|q| q.0 == 'X').map(|q| q.1);
        let ny = words.iter().find(|q| q.0 == 'Y').map(|q| q.1);
        let nz = words.iter().find(|q| q.0 == 'Z').map(|q| q.1);
        if nx.is_none() && ny.is_none() && nz.is_none() {
            continue; // modal-only block (G4 dwell, M-code, feed change)
        }
        let prev = match pos {
            Some(p) => p,
            None => [nx.unwrap_or(0.0), ny.unwrap_or(0.0), nz.unwrap_or(0.0)],
        };
        let next = [
            nx.unwrap_or(prev[0]),
            ny.unwrap_or(prev[1]),
            nz.unwrap_or(prev[2]),
        ];
        segments.push(Seg3 {
            from: prev,
            to: next,
            cut: motion == 1,
            feed,
        });
        pos = Some(next);
    }

    // ---- swept tool solids ---------------------------------------------------
    let mut parts: Vec<Manifold> = Vec::new();
    let mut rapids = 0usize;
    let mut cut_xy = 0usize;
    let mut cut_z = 0usize;
    for s in &segments {
        if !s.cut {
            rapids += 1;
            continue;
        }
        let (dx, dy) = (s.to[0] - s.from[0], s.to[1] - s.from[1]);
        let a2 = [s.from[0], s.from[1]];
        let b2 = [s.to[0], s.to[1]];
        if dx.abs() > 1e-12 || dy.abs() > 1e-12 {
            cut_xy += 1;
            let poly = stadium(a2, b2, tool.radius, 24);
            prismatic(
                &mut parts,
                prism_from_polygon(&poly, s.to[2], s.to[2] + tool.length)?,
            );
            if (s.to[2] - s.from[2]).abs() > 1e-12 {
                // diagonal cut: also sweep the vertical component (never
                // emitted by our planner; honest coverage for foreign input)
                let disc = stadium(a2, a2, tool.radius, 24);
                prismatic(
                    &mut parts,
                    prism_from_polygon(&disc, s.to[2], s.from[2].max(s.to[2] + tool.length))?,
                );
            }
        } else {
            cut_z += 1;
            let disc = stadium(a2, a2, tool.radius, 24);
            let z0 = s.to[2];
            let z1 = s.from[2].max(s.to[2] + tool.length);
            prismatic(&mut parts, prism_from_polygon(&disc, z0, z1)?);
        }
    }
    let worn = union_all(parts)?;
    let stock_m = csg::manifold_from_display(&stock.mesh)
        .map_err(|e| CamError(format!("sim stock manifold: {e}")))?;
    let pocket_m = {
        let pm = extrude_region(pocket, z_floor, z_top)
            .map_err(|e| CamError(format!("pocket prism: {e}")))?;
        pm.closed_manifold_check()
            .map_err(|e| CamError(format!("pocket prism failed manifold audit: {e}")))?;
        csg::manifold_from_mesh64(&pm).map_err(|e| CamError(format!("pocket prism: {e}")))?
    };
    let removed = stock_m.intersection(&worn);
    let inside = removed.intersection(&pocket_m);
    let sim_removed_mm3 = removed.volume();
    let inside_mm3 = inside.volume();
    let stock_mm3 = stock_m.volume();
    let pocket_mm3 = stock_mm3 - stock_m.difference(&pocket_m).volume();

    Ok(SimReport {
        segments,
        rapids,
        cut_xy,
        cut_z,
        stock_mm3,
        pocket_mm3,
        removed_mm3: sim_removed_mm3,
        inside_mm3,
        gouge_mm3: sim_removed_mm3 - inside_mm3,
        uncut_mm3: pocket_mm3 - inside_mm3,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg::{FeatureId, GeoNode, Timeline, evaluate};

    /// One straight G1 cut across a 20x20x5 slab with a 2.5 r tool:
    /// sim removal must equal the analytic stadium x depth (the pocket
    /// prism is exactly the swept band) within arc-chord slop.
    #[test]
    fn straight_cut_matches_boolean_ground_truth() {
        let mut tl = Timeline::default();
        tl.push(
            "slab",
            GeoNode::Box {
                w: 30.0,
                h: 12.0,
                d: 5.0,
            },
        )
        .unwrap();
        let solid = evaluate(&tl).unwrap();
        // box is centered (top at z=2.5); the band stays inside the
        // footprint so caps at x=±10.5 never clip at a wall
        let text = "\
G21 G90 G17 G94 G40
M3 S8000
G4 P0.5
G0 X-8.000 Y0.000 Z7.500
G1 Z0.500 F600.0
G1 X8.000 Y0.000 F600.0
G0 Z7.500
M5
M2
";
        let pocket = stadium([-8.0, 0.0], [8.0, 0.0], 2.5, 48);
        let pocket = Region2 {
            outer: pocket,
            holes: vec![],
        };
        let rep = replay(
            text,
            &SimTool {
                radius: 2.5,
                length: 20.0,
            },
            &solid,
            &pocket,
            2.5,
            0.5,
        )
        .unwrap();
        assert_eq!(rep.cut_xy, 1);
        assert_eq!(rep.cut_z, 1);
        // ground-truth prism == swept band: exact stadium (48-step caps),
        // sim tool caps at 24 steps -> diff is only cap chord error.
        // stadium volume = (L*2r + pi r^2) * depth
        let band = (16.0 * 5.0 + std::f64::consts::PI * 2.5 * 2.5) * 2.0;
        assert!(
            (rep.removed_mm3 - band).abs() / band < 0.02,
            "stadium vs band: {} vs {band}",
            rep.removed_mm3
        );
        assert!(
            rep.gouge_mm3 < 0.01 * band,
            "straight cut gouge {}",
            rep.gouge_mm3
        );
    }

    #[test]
    fn replay_rejects_out_of_subset_dialect() {
        let mut tl = Timeline::default();
        tl.push(
            "s",
            GeoNode::Box {
                w: 1.0,
                h: 1.0,
                d: 1.0,
            },
        )
        .unwrap();
        let solid = evaluate(&tl).unwrap();
        let pocket = Region2 {
            outer: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            holes: vec![],
        };
        let tool = SimTool {
            radius: 0.2,
            length: 2.0,
        };
        for bad in [
            "G20\nM3 S1000\nG1 X0.100 Y0.100 F100.0\nM5\nM2\n",
            "G21 G91\nM3 S1000\nG1 X0.100 Y0.100 F100.0\nM5\nM2\n",
            "G21 G18\nM3 S1000\nG1 X0.100 Y0.100 F100.0\nM5\nM2\n",
            "G21\nM3 S1000\nG2 X0.100 Y0.100 I0.050 J0.050 F100.0\nM5\nM2\n",
            "G21\nT1\nM3 S1000\nG1 X0.100 Y0.100 F100.0\nM5\nM2\n",
        ] {
            assert!(
                replay(bad, &tool, &solid, &pocket, 0.5, -0.5).is_err(),
                "replay must reject: {bad}"
            );
        }
        let _ = FeatureId(1); // keep import used in scope form
    }

    #[test]
    fn stadium_area_matches_analytic() {
        let pts = stadium([0.0, 0.0], [10.0, 0.0], 2.0, 64);
        let area: f64 = pts
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let q = pts[(i + 1) % pts.len()];
                p[0] * q[1] - q[0] * p[1]
            })
            .sum::<f64>()
            / 2.0;
        // exact stadium area = 2r*L + pi r^2; 64-step caps approximate
        let exact = 2.0 * 2.0 * 10.0 + std::f64::consts::PI * 4.0;
        assert!((area - exact).abs() / exact < 1e-3, "{area} vs {exact}");
        assert!(area > 0.0, "CCW");
    }
}
