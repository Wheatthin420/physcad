//! M7 pocket planner: contour-parallel offset rings from a rasterized
//! sketch region, with islands (holes) handled by the same distance
//! field, plus explicit self-certification (coverage + wall clearance)
//! before any moves leave the planner.
//!
//! Algorithm (docs/CAM.md §planner):
//! 1. Rasterize the region (point-in-even-odd-polygon) at `pixel` mm.
//! 2. Euclidean distance transform (Felzenszwalb two-pass squared EDT):
//!    `t(p)` = distance from each interior pixel center to the nearest
//!    exterior sample (region wall OR island).
//! 3. Cutter-center ring levels at `t = r + k*(2r - stepover)`: ring
//!    centers stay `r` inside the wall (tangent cut, no gouge), spacing
//!    `2r - stepover` guarantees lateral overlap `stepover` between
//!    adjacent passes; a core level at `max_t - 0.4r` is appended when
//!    the medial axis lies deeper than one coverage radius.
//! 4. Marching squares (inside-left directed segments) -> closed loops,
//!    oriented by construction (region on the left: outer rings CCW,
//!    island rings CW).
//! 5. Ramer-Douglas-Peucker simplification to `chord` mm (bounds wall
//!    placement error together with pixel quantization).
//! 6. Rings are chained into continuous cut paths by bridges; every
//!    bridge is SAMPLED and rejected unless full tool radius of
//!    clearance exists along it (bridges crossing an island or pinching
//!    a wall are hard errors, not silent gouges).
//! 7. Self-certification: an EDT from the cut paths proves every region
//!    pixel lies within `r + 2*pixel` of some pass, or planning fails.
//!
//! This is contour-parallel roughing — "adaptive" in the sense that
//! paths follow the part geometry (islands included), not a fixed grid.
//! Trochoidal/high-efficiency rest roughing is deferred (docs/CAM.md).

use std::collections::HashMap;

use super::CamError;
use super::gcode::Motion;
use crate::csg::Solid;
use crate::naming::Surface;
use crate::sketch::loops::Region2;

#[derive(Clone, Copy, Debug)]
pub struct PocketParams {
    /// end mill radius, mm
    pub tool_radius: f64,
    /// lateral overlap between adjacent ring passes, mm (< 2r)
    pub stepover: f64,
    /// axial depth of one pass, mm
    pub depth_per_pass: f64,
    /// raster resolution, mm
    pub pixel: f64,
    /// polyline simplification chord tolerance, mm
    pub chord: f64,
    /// rapid clearance height above the stock top, mm
    pub z_clear: f64,
}

#[derive(Clone, Debug)]
pub struct RingLevel {
    pub t: f64,
    pub loops: Vec<Vec<[f64; 2]>>,
    /// bridge segments added to chain loops within/across levels
    pub links: Vec<[[f64; 2]; 2]>,
}

#[derive(Clone, Debug)]
pub struct PocketPlan {
    pub top_face: String,
    pub z_top: f64,
    pub z_floor: f64,
    pub levels: Vec<RingLevel>,
    /// tool tip Z at each depth pass (descending)
    pub pass_tips: Vec<f64>,
    pub moves: Vec<Motion>,
    /// worst (distance-to-path - tool radius) over the region raster, mm
    pub coverage_excess_mm: f64,
    /// footprint area no r-disc inside the region can reach (sharp
    /// corners; rest machining deferred) in mm^2
    pub rest_area_mm2: f64,
    pub cut_moves: usize,
    pub rapid_moves: usize,
}

/// Even-odd point-in-region across the outer loop and every hole.
pub fn point_in_region(r: &Region2, p: [f64; 2]) -> bool {
    let inside = |poly: &[[f64; 2]]| -> bool {
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
    };
    let mut c = inside(&r.outer);
    for h in &r.holes {
        if inside(h) {
            c = !c;
        }
    }
    c
}

struct Raster {
    x0: f64,
    y0: f64,
    w: usize,
    h: usize,
    p: f64,
    mask: Vec<bool>,
}

fn raster(region: &Region2, p: f64, margin_px: usize) -> Raster {
    let (mut minx, mut miny, mut maxx, mut maxy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for l in std::iter::once(&region.outer).chain(region.holes.iter()) {
        for q in l {
            minx = minx.min(q[0]);
            maxx = maxx.max(q[0]);
            miny = miny.min(q[1]);
            maxy = maxy.max(q[1]);
        }
    }
    let x0 = minx - margin_px as f64 * p;
    let y0 = miny - margin_px as f64 * p;
    let w = ((maxx - x0) / p).ceil() as usize + margin_px + 1;
    let h = ((maxy - y0) / p).ceil() as usize + margin_px + 1;
    let mask = (0..h * w)
        .map(|idx| {
            let (gx, gy) = ((idx % w) as f64, (idx / w) as f64);
            point_in_region(region, [x0 + (gx + 0.5) * p, y0 + (gy + 0.5) * p])
        })
        .collect();
    Raster {
        x0,
        y0,
        w,
        h,
        p,
        mask,
    }
}

fn center(r: &Raster, gx: usize, gy: usize) -> [f64; 2] {
    [
        r.x0 + (gx as f64 + 0.5) * r.p,
        r.y0 + (gy as f64 + 0.5) * r.p,
    ]
}

/// Squared Euclidean distance transform (Felzenszwalb & Huttenlocher,
/// 2-pass lower envelope of parabolas). Seeds (distance 0) are pixels
/// where `seed` is true; returns squared distance in mm^2.
fn edt2(w: usize, h: usize, p: f64, seed: &[bool]) -> Vec<f64> {
    const INF: f64 = f64::MAX / 4.0;
    let mut f = vec![INF; w * h];
    for (i, s) in seed.iter().enumerate() {
        if *s {
            f[i] = 0.0;
        }
    }
    fn edt1d(f: &[f64], out: &mut [f64], d: f64) {
        let n = f.len();
        let mut v = vec![0usize; n];
        let mut z = vec![0f64; n + 1];
        let mut k = 0usize;
        v[0] = 0;
        z[0] = f64::NEG_INFINITY;
        z[1] = f64::INFINITY;
        for q in 1..n {
            let mut s = ((f[q] + (q as f64 * d).powi(2)) - (f[v[k]] + (v[k] as f64 * d).powi(2)))
                / (2.0 * d * d * (q - v[k]) as f64);
            while s <= z[k] && k > 0 {
                k -= 1;
                s = ((f[q] + (q as f64 * d).powi(2)) - (f[v[k]] + (v[k] as f64 * d).powi(2)))
                    / (2.0 * d * d * (q - v[k]) as f64);
            }
            k += 1;
            v[k] = q;
            z[k] = s;
            z[k + 1] = f64::INFINITY;
        }
        k = 0;
        for (q, o) in out.iter_mut().enumerate() {
            // z holds intersections in INDEX space (u = x/d): compare to q,
            // not q*d — mixing the two spaces was a real FH transcription bug
            while z[k + 1] < q as f64 {
                k += 1;
            }
            let dv = (q as f64 - v[k] as f64) * d;
            *o = f[v[k]] + dv * dv;
        }
    }
    // rows then columns
    let mut g = vec![0f64; w * h];
    let mut col = vec![0f64; h];
    let mut row = vec![0f64; w];
    for gy in 0..h {
        let (a, b) = (gy * w, gy * w + w);
        edt1d(&f[a..b], &mut row, p);
        for (gx, val) in row.iter().enumerate() {
            g[gy * w + gx] = *val;
        }
    }
    let mut out = vec![0f64; w * h];
    let mut colout = vec![0f64; h];
    for gx in 0..w {
        for (gy, c) in col.iter_mut().enumerate() {
            *c = g[gy * w + gx];
        }
        edt1d(&col, &mut colout, p);
        for (gy, val) in colout.iter().enumerate() {
            out[gy * w + gx] = *val;
        }
    }
    out
}

/// Marching squares at `level` over the distance field, inside-left:
/// the region `{t >= level}` stays on the LEFT of every directed
/// segment, so outer rings come out CCW and island rings CW.
fn contours(
    field: &[f64],
    w: usize,
    h: usize,
    level: f64,
    x0: f64,
    y0: f64,
    p: f64,
) -> Result<Vec<Vec<[f64; 2]>>, CamError> {
    // crossing node = the lattice edge it sits on, as ordered corner pair
    type Node = (usize, usize, usize, usize);
    let mut pt: HashMap<Node, [f64; 2]> = HashMap::new();
    let mut segs: Vec<(Node, Node)> = Vec::new();
    let cross =
        &mut |a: usize, b: usize, va: f64, vb: f64, ax: usize, ay: usize, bx: usize, by: usize| {
            let _ = (a, b); // lattice indices (kept for readability of the call sites)
            let key = if (ax, ay) < (bx, by) {
                (ax, ay, bx, by)
            } else {
                (bx, by, ax, ay)
            };
            let u = (level - va) / (vb - va);
            // lattice node (gx,gy) sits at pixel center (x0+(gx+0.5)p, ...):
            // contour points are WORLD millimetres, not grid indices
            let q = pt.entry(key).or_insert([
                x0 + (ax as f64 + 0.5 + (bx - ax) as f64 * u) * p,
                y0 + (ay as f64 + 0.5 + (by - ay) as f64 * u) * p,
            ]);
            (*q, key)
        };
    for gy in 0..h - 1 {
        for gx in 0..w - 1 {
            let i00 = gy * w + gx;
            let i10 = i00 + 1;
            let i11 = i00 + w + 1;
            let i01 = i00 + w;
            let (v00, v10, v11, v01) = (field[i00], field[i10], field[i11], field[i01]);
            let code = (v00 >= level) as u8
                + 2 * (v10 >= level) as u8
                + 4 * (v11 >= level) as u8
                + 8 * (v01 >= level) as u8;
            if code == 0 || code == 15 {
                continue;
            }
            // lattice corners and edge crossing points
            let c00 = (gx, gy);
            let c10 = (gx + 1, gy);
            let c11 = (gx + 1, gy + 1);
            let c01 = (gx, gy + 1);
            let mut e = None; // bottom c00-c10
            let mut s = None; // right  c10-c11
            let mut we = None; // top   c01-c11
            let mut n = None; // left  c00-c01
            if (v00 >= level) != (v10 >= level) {
                e = Some(cross(i00, i10, v00, v10, c00.0, c00.1, c10.0, c10.1).1);
            }
            if (v10 >= level) != (v11 >= level) {
                s = Some(cross(i10, i11, v10, v11, c10.0, c10.1, c11.0, c11.1).1);
            }
            if (v01 >= level) != (v11 >= level) {
                we = Some(cross(i01, i11, v01, v11, c01.0, c01.1, c11.0, c11.1).1);
            }
            if (v00 >= level) != (v01 >= level) {
                n = Some(cross(i00, i01, v00, v01, c00.0, c00.1, c01.0, c01.1).1);
            }
            // directed segments, interior ({t>=level}) on the LEFT;
            // a crossing exists only on edges that actually change side
            let need = |o: Option<Node>| {
                o.ok_or_else(|| CamError(format!("marching squares: missing edge (code {code})")))
            };
            match code {
                1 => segs.push((need(e)?, need(n)?)),
                2 => segs.push((need(s)?, need(e)?)),
                3 => segs.push((need(s)?, need(n)?)),
                4 => segs.push((need(we)?, need(s)?)),
                5 | 10 => {
                    let center_in = 0.25 * (v00 + v10 + v11 + v01) >= level;
                    let (e, s, we, n) = (need(e)?, need(s)?, need(we)?, need(n)?);
                    match (code, center_in) {
                        (5, true) => {
                            segs.push((e, s));
                            segs.push((we, n));
                        }
                        (5, false) => {
                            segs.push((e, n));
                            segs.push((we, s));
                        }
                        (_, true) => {
                            segs.push((n, e));
                            segs.push((s, we));
                        }
                        (_, false) => {
                            segs.push((s, e));
                            segs.push((n, we));
                        }
                    }
                }
                6 => segs.push((need(we)?, need(e)?)),
                7 => segs.push((need(we)?, need(n)?)),
                8 => segs.push((need(n)?, need(we)?)),
                9 => segs.push((need(e)?, need(we)?)),
                11 => segs.push((need(s)?, need(we)?)),
                12 => segs.push((need(n)?, need(s)?)),
                13 => segs.push((need(e)?, need(s)?)),
                14 => segs.push((need(n)?, need(e)?)),
                _ => return Err(CamError(format!("marching squares: bad code {code}"))),
            }
        }
    }
    // stitch directed segments into closed loops
    let mut start_at: HashMap<Node, usize> = HashMap::new();
    for (i, (a, _)) in segs.iter().enumerate() {
        if start_at.insert(*a, i).is_some() {
            return Err(CamError(
                "marching squares: node with two outgoing edges".into(),
            ));
        }
    }
    let pos = |node: &Node| pt[node];
    let mut used = vec![false; segs.len()];
    let mut loops: Vec<Vec<[f64; 2]>> = Vec::new();
    for si in 0..segs.len() {
        if used[si] {
            continue;
        }
        let mut loop_pts = vec![pos(&segs[si].0)];
        let mut cur = si;
        loop {
            used[cur] = true;
            let end = segs[cur].1;
            loop_pts.push(pos(&end));
            match start_at.get(&end) {
                Some(&nx) if nx == si => break, // closed back to the first segment
                Some(&nx) if !used[nx] => cur = nx,
                Some(_) => {
                    return Err(CamError(
                        "marching squares: open or colliding contour".into(),
                    ));
                }
                None => {
                    return Err(CamError(format!(
                        "marching squares: contour not closed at node {end:?}"
                    )));
                }
            }
        }
        loop_pts.pop(); // implicit wraparound vertex
        if loop_pts.len() >= 3 {
            loops.push(loop_pts);
        }
    }
    Ok(loops)
}

/// Ramer-Douglas-Peucker on an open polyline.
fn rdp(pts: &[[f64; 2]], eps: f64) -> Vec<[f64; 2]> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let mut imax = 0usize;
    let mut dmax = -1.0;
    for (i, p) in pts.iter().enumerate().take(pts.len() - 1).skip(1) {
        let d2 = if l2 == 0.0 {
            let ux = p[0] - a[0];
            let uy = p[1] - a[1];
            ux * ux + uy * uy
        } else {
            let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0);
            let qx = a[0] + t * dx - p[0];
            let qy = a[1] + t * dy - p[1];
            qx * qx + qy * qy
        };
        if d2 > dmax {
            dmax = d2;
            imax = i;
        }
    }
    if dmax > eps * eps {
        let mut left = rdp(&pts[..=imax], eps);
        let right = rdp(&pts[imax..], eps);
        left.extend(right[1..].iter());
        left
    } else {
        vec![a, b]
    }
}

fn rotated(loop_pts: &[[f64; 2]], start: usize) -> Vec<[f64; 2]> {
    let n = loop_pts.len();
    (0..n).map(|i| loop_pts[(start + i) % n]).collect()
}

/// Plan an adaptive (offset-ring) pocket.
///
/// * `stock`     — evaluated solid; `top_face` must be one of its named faces.
/// * `top_face`  — M6 face key whose declaring plane is the pocket top (z_top).
/// * `region`    — pocket footprint in XY (sketch region; holes = islands).
/// * `depth`     — pocket depth below the top face, mm.
pub fn plan_pocket(
    stock: &Solid,
    top_face: &str,
    region: &Region2,
    params: &PocketParams,
    depth: f64,
) -> Result<PocketPlan, CamError> {
    // ---- face binding (M6) ------------------------------------------------
    let rec = stock
        .table
        .iter()
        .find(|r| r.key() == top_face)
        .ok_or_else(|| CamError(format!("no face record with key {top_face:?}")))?;
    if stock.face(top_face).is_none() {
        return Err(CamError(format!(
            "face {top_face:?} is declared but has no triangles on the evaluated solid"
        )));
    }
    let z_top = match rec.surface {
        Surface::Plane { n, off } if n[2].abs() > 0.999 => off / n[2],
        Surface::Plane { n, off } => {
            return Err(CamError(format!(
                "pocket top face {top_face:?} is not horizontal: plane {:?} offset {off}",
                n
            )));
        }
        _ => {
            return Err(CamError(format!(
                "pocket top face {top_face:?} must be a plane"
            )));
        }
    };
    let z_floor = z_top - depth;

    let p = params.pixel;
    let r = params.tool_radius;
    if r <= 0.0 || params.stepover <= 0.0 || params.stepover >= 2.0 * r {
        return Err(CamError(
            "tool radius and stepover must satisfy 0 < stepover < 2 * radius".into(),
        ));
    }
    if depth <= 0.0 || params.depth_per_pass <= 0.0 {
        return Err(CamError("pocket and pass depths must be positive".into()));
    }

    // ---- raster + distance field ------------------------------------------
    let rast = raster(region, p, 3);
    let n_px = rast.w * rast.h;
    let seed = (0..n_px).map(|i| !rast.mask[i]).collect::<Vec<_>>();
    let d2 = edt2(rast.w, rast.h, p, &seed);
    let field: Vec<f64> = d2.iter().map(|v| v.max(0.0).sqrt()).collect();
    let max_t = field
        .iter()
        .enumerate()
        .filter(|(i, _)| rast.mask[*i])
        .map(|(_, v)| *v)
        .fold(0.0f64, f64::max);
    if max_t < r {
        return Err(CamError(format!(
            "cutter radius {r} mm does not fit: narrowest pocket passage is only {:.2} mm half-width; \
             use a smaller tool (rest roughing not implemented)",
            max_t
        )));
    }

    // ---- ring levels (with bounded coverage-driven refinement) ------------
    // Contour chains from the wall at spacing 2r - stepover, plus a core
    // level. Spacing alone does NOT guarantee coverage where an inner
    // contour shrinks to a blob and stops wrapping a strip (reflex
    // corners do this); the coverage self-cert catches those and we
    // insert a corrective level through the failing point and rebuild,
    // bounded to keep planning deterministic and fast.
    let spacing = 2.0 * r - params.stepover;
    let mut levels_t: Vec<f64> = Vec::new();
    {
        let mut t = r;
        while t <= max_t {
            levels_t.push(t);
            t += spacing;
        }
        if let Some(&last) = levels_t.last()
            && max_t - last > r
        {
            levels_t.push(max_t - 0.4 * r);
        }
    }

    // contours + simplification + greedy ring order for a level set
    let build = |levels_t: &[f64]| -> Result<Vec<RingLevel>, CamError> {
        let mut levels: Vec<RingLevel> = Vec::new();
        for &lv in levels_t {
            let mut rl = RingLevel {
                t: lv,
                loops: Vec::new(),
                links: Vec::new(),
            };
            let mut loops = contours(&field, rast.w, rast.h, lv, rast.x0, rast.y0, rast.p)?
                .iter()
                .map(|lp| {
                    // rotate to the max-(x,y) vertex, RDP as an open chain
                    let mut start = 0usize;
                    for i in 0..lp.len() {
                        if lp[i][0] > lp[start][0]
                            || (lp[i][0] == lp[start][0] && lp[i][1] > lp[start][1])
                        {
                            start = i;
                        }
                    }
                    let rot = rotated(lp, start);
                    let mut open = rot.clone();
                    open.push(rot[0]);
                    let mut simp = rdp(&open, params.chord);
                    simp.pop();
                    simp
                })
                .filter(|lp| lp.len() >= 3)
                .collect::<Vec<_>>();
            if loops.is_empty() {
                continue; // level with no material left to hug
            }
            // start with the loop holding the global max-x vertex
            loops.sort_by(|a, b| {
                let ax = a.iter().map(|q| q[0]).fold(f64::MIN, f64::max);
                let bx = b.iter().map(|q| q[0]).fold(f64::MIN, f64::max);
                bx.total_cmp(&ax)
            });
            let mut ordered: Vec<Vec<[f64; 2]>> = vec![loops.remove(0)];
            while !loops.is_empty() {
                // greedy nearest-ring order; the actual bridge-vs-retract
                // decision happens at path build, where clearance is tested
                let cur = *ordered.last().unwrap().last().unwrap();
                let mut best: Option<(f64, usize)> = None;
                for (i, lp) in loops.iter().enumerate() {
                    let mut bd = f64::MAX;
                    for q in lp {
                        let d = (q[0] - cur[0]).powi(2) + (q[1] - cur[1]).powi(2);
                        if d < bd {
                            bd = d;
                        }
                    }
                    if best.is_none_or(|(b, _)| bd < b) {
                        best = Some((bd, i));
                    }
                }
                let (_, i) = best.unwrap();
                ordered.push(loops.remove(i));
            }
            rl.loops = ordered;
            levels.push(rl);
        }
        Ok(levels)
    };

    if build(&levels_t)?.is_empty() {
        return Err(CamError("pocket produced no cutting rings".into()));
    }

    // distance to the eroded set: which pixels can ANY r-disc touch?
    let erode_seed: Vec<bool> = (0..n_px).map(|i| field[i] >= r).collect();
    let ed2 = edt2(rast.w, rast.h, p, &erode_seed);

    // (worst excess beyond r, index of worst pixel, rest pixel count)
    // measured against the ring loops only — bridges only ever add cut
    // samples, so this is conservative for the final path.
    let coverage = |levels: &[RingLevel]| -> Result<(f64, usize, u64), CamError> {
        let mut cseed = vec![false; n_px];
        let mark = |a: [f64; 2], b: [f64; 2], cseed: &mut Vec<bool>| {
            let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
            let steps = (len / (p * 0.5)).ceil().max(1.0) as usize;
            for k in 0..=steps {
                let t = k as f64 / steps as f64;
                let q = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
                let gx = ((q[0] - rast.x0) / p) as i64;
                let gy = ((q[1] - rast.y0) / p) as i64;
                if gx >= 0 && gy >= 0 && (gx as usize) < rast.w && (gy as usize) < rast.h {
                    cseed[(gy as usize) * rast.w + gx as usize] = true;
                }
            }
        };
        for lv in levels {
            for lp in &lv.loops {
                for i in 0..lp.len() {
                    mark(lp[i], lp[(i + 1) % lp.len()], &mut cseed);
                }
            }
        }
        if !cseed.iter().any(|s| *s) {
            return Err(CamError("coverage check found no cut samples".into()));
        }
        let cd2 = edt2(rast.w, rast.h, p, &cseed);
        let mut worst = (0.0f64, 0usize);
        let mut rest_px = 0u64;
        for i in 0..n_px {
            if !rast.mask[i] {
                continue;
            }
            let reach = ed2[i].max(0.0).sqrt();
            if reach > r + p {
                rest_px += 1;
                continue;
            }
            let excess = cd2[i].max(0.0).sqrt() - r;
            if excess > worst.0 {
                worst = (excess, i);
            }
        }
        Ok((worst.0, worst.1, rest_px))
    };

    let mut levels = build(&levels_t)?;
    let mut refine_msg = String::new();
    for attempt in 0..6u32 {
        let (worst, wi, _rest) = coverage(&levels)?;
        if worst <= 2.0 * p {
            break;
        }
        if attempt == 5 {
            let (gx, gy) = (wi % rast.w, wi / rast.w);
            let q = center(&rast, gx, gy);
            return Err(CamError(format!(
                "pocket coverage gap at ({:.2}, {:.2}): {:.3} mm beyond tool radius \
                 (budget {:.3} mm) after {attempt} level refinements (levels {:?}, max_t {:.3})",
                q[0],
                q[1],
                worst,
                2.0 * p,
                levels_t,
                max_t,
            )));
        }
        // insert a contour level just inside the failing pixel's t: it
        // wraps the offending strip and cannot duplicate an existing one
        let mut nl = field[wi].max(r) - 0.05 * r;
        while levels_t.iter().any(|c| (c - nl).abs() < 3.0 * p) {
            nl -= 4.0 * p;
        }
        if nl < 0.5 * r {
            refine_msg = format!("refinement floor reached at t={nl:.3}");
        }
        levels_t.push(nl);
        levels_t.sort_by(f64::total_cmp);
        levels = build(&levels_t)?;
    }
    if !refine_msg.is_empty() {
        return Err(CamError(format!(
            "coverage refinement could not make progress: {refine_msg}"
        )));
    }

    // ---- moves ---------------------------------------------------------------
    let z_clear = z_top + params.z_clear;
    let n_pass = (depth / params.depth_per_pass).ceil().max(1.0) as usize;
    let ap = depth / n_pass as f64;
    let pass_tips: Vec<f64> = (1..=n_pass).map(|k| z_top - k as f64 * ap).collect();

    // ordered point sequence for one depth pass: each entry is
    // (xy, needs_retract_and_plunge). In-plane bridges are used only
    // when full tool clearance exists along them (docs/CAM.md §linking).
    let mut path: Vec<([f64; 2], bool)> = Vec::new();
    let mut prev_end: Option<[f64; 2]> = None;
    for lv in &mut levels {
        for lp in &mut lv.loops {
            let mut ring = std::mem::take(lp);
            let mut jump = true;
            if let Some(pe) = prev_end {
                let mut best: Option<(f64, usize)> = None;
                for (j, q) in ring.iter().enumerate() {
                    if !clearance_ok(&rast, &field, r, [pe, *q]) {
                        continue;
                    }
                    let d = (q[0] - pe[0]).powi(2) + (q[1] - pe[1]).powi(2);
                    if best.is_none_or(|(bd, _)| d < bd) {
                        best = Some((d, j));
                    }
                }
                if let Some((_, j)) = best {
                    ring = rotated(&ring, j);
                    lv.links.push([pe, ring[0]]);
                    jump = false;
                }
            }
            path.push((ring[0], jump));
            for q in &ring[1..] {
                path.push((*q, false));
            }
            path.push((ring[0], false)); // close the ring on itself
            prev_end = Some(ring[0]);
            *lp = ring;
        }
    }

    let mut moves: Vec<Motion> = Vec::new();
    for tip in &pass_tips {
        for (k, (q, jump)) in path.iter().enumerate() {
            if *jump || k == 0 {
                moves.push(Motion::Rapid {
                    x: q[0],
                    y: q[1],
                    z: z_clear,
                });
                moves.push(Motion::Cut {
                    x: q[0],
                    y: q[1],
                    z: *tip,
                });
            } else {
                moves.push(Motion::Cut {
                    x: q[0],
                    y: q[1],
                    z: *tip,
                });
            }
        }
    }
    let last = path.last().map(|p| p.0).unwrap_or([0.0, 0.0]);
    moves.push(Motion::Rapid {
        x: last[0],
        y: last[1],
        z: z_clear,
    });

    // ---- final certification numbers ----------------------------------------
    // (the refinement loop above already proved reachable-pixel coverage
    // against rings only; bridges add samples, so loops-only stays the
    // conservative statement of record — see also the sim-vs-CSG gate)
    let (cov_excess, _wi, rest_px) = coverage(&levels)?;
    if cov_excess > 2.0 * p {
        return Err(CamError(format!(
            "internal: coverage regressed after path build ({cov_excess:.3} mm > {:.3})",
            2.0 * p
        )));
    }
    let rest_area_mm2 = rest_px as f64 * p * p;

    let cut_moves = moves.iter().filter(|m| m.is_cut()).count();
    let rapid_moves = moves.len() - cut_moves;
    Ok(PocketPlan {
        top_face: top_face.to_string(),
        z_top,
        z_floor,
        levels,
        pass_tips,
        moves,
        coverage_excess_mm: cov_excess,
        rest_area_mm2,
        cut_moves,
        rapid_moves,
    })
}

/// True if the whole segment keeps full tool clearance (EDT >= r - 2p at
/// every sample; samples every half pixel).
fn clearance_ok(rast: &Raster, field: &[f64], r: f64, link: [[f64; 2]; 2]) -> bool {
    let (a, b) = (link[0], link[1]);
    let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
    if len <= 1e-12 {
        return true;
    }
    let steps = (len / (rast.p * 0.5)).ceil() as usize;
    for k in 0..=steps {
        let t = k as f64 / steps as f64;
        let q = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        let gx = ((q[0] - rast.x0) / rast.p) as i64;
        let gy = ((q[1] - rast.y0) / rast.p) as i64;
        let ok = gx >= 0
            && gy >= 0
            && (gx as usize) < rast.w
            && (gy as usize) < rast.h
            && field[(gy as usize) * rast.w + gx as usize] >= r - 2.0 * rast.p;
        if !ok {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square_region(half: f64) -> Region2 {
        Region2 {
            outer: vec![[-half, -half], [half, -half], [half, half], [-half, half]],
            holes: vec![],
        }
    }

    #[test]
    fn point_in_region_even_odd_with_hole() {
        let mut r = square_region(10.0);
        r.holes
            .push(vec![[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]]);
        assert!(point_in_region(&r, [0.0, 6.0]));
        assert!(!point_in_region(&r, [0.0, 0.0])); // inside island
        assert!(!point_in_region(&r, [20.0, 0.0]));
        assert!(!point_in_region(&r, [-11.0, 0.0]));
    }

    #[test]
    fn edt_of_full_disk_matches_radius() {
        // mask = square 21x21 all true except border ring outside r=10
        let w = 21usize;
        let h = 21usize;
        let p = 1.0;
        let seed: Vec<bool> = (0..h * w)
            .map(|i| {
                let x = (i % w) as f64 - 10.0;
                let y = (i / w) as f64 - 10.0;
                x * x + y * y > 100.0
            })
            .collect();
        let d2 = edt2(w, h, p, &seed);
        // center pixel distance ~ 10 +- one half-pixel quantization
        let c = d2[10 * w + 10].sqrt();
        assert!((c - 10.0).abs() < 1.0, "center distance {c}");
        // pixel on the seed boundary is 0
        assert_eq!(d2[0], 0.0);
    }

    #[test]
    fn contours_of_square_field_are_closed() {
        let w = 41usize;
        let h = 41usize;
        let seed: Vec<bool> = (0..h * w)
            .map(|i| {
                let gx = i % w;
                let gy = i / w;
                !((10..31).contains(&gx)) || !((10..31).contains(&gy))
            })
            .collect();
        let d2 = edt2(w, h, 1.0, &seed);
        let field: Vec<f64> = d2.iter().map(|v| v.max(0.0).sqrt()).collect();
        let loops = contours(&field, w, h, 5.0, -0.5, -0.5, 1.0).expect("contours");
        assert_eq!(loops.len(), 1);
        assert!(loops[0].len() >= 4);
        // CCW (positive area) for the outer boundary
        let a = loops[0]
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let q = loops[0][(i + 1) % loops[0].len()];
                p[0] * q[1] - q[0] * p[1]
            })
            .sum::<f64>()
            / 2.0;
        assert!(a > 0.0, "outer contour must be CCW, got area {a}");
    }

    #[test]
    fn rdp_keeps_shape_and_collapses_straight_runs() {
        let pts: Vec<[f64; 2]> = (0..100).map(|i| [i as f64 * 0.1, 0.0]).collect();
        let s = rdp(&pts, 1e-9);
        assert_eq!(s.len(), 2);
        let arc: Vec<[f64; 2]> = (0..=40)
            .map(|i| {
                let a = std::f64::consts::PI * i as f64 / 40.0;
                [a.cos(), a.sin()]
            })
            .collect();
        let s = rdp(&arc, 5e-2);
        assert!(s.len() > 2 && s.len() < arc.len(), "got {}", s.len());
    }

    /// Exhaustive closure probe: a 4x4 checker patch embedded in a
    /// uniform field makes every crossing node interior (shared by two
    /// cells), so the directed table must stitch into closed loops for
    /// ALL codes that appear — saddle pairings (5/10 with center avg ==
    /// level -> the center_in branch), single corners, and bands alike.
    /// A table error shows up as an open/colliding contour error.
    #[test]
    fn contours_close_on_checker_patch_and_every_reachable_code() {
        let n = 10usize;
        let field: Vec<f64> = (0..n * n)
            .map(|i| {
                let (x, y) = (i % n, i / n);
                if (2..=5).contains(&x) && (2..=5).contains(&y) && (x + y) % 2 == 0 {
                    1.0
                } else {
                    2.0
                }
            })
            .collect();
        let loops = contours(&field, n, n, 1.5, -0.5, -0.5, 1.0).expect("contours must close");
        assert!(!loops.is_empty());
    }
}
