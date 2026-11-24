//! M5: closed sketch regions -> closed solids (extrude / revolve).
//!
//! Mesh-based, no B-rep (the migration decision point stays documented in
//! docs/MILESTONES.md). Each builder returns a `Mesh64` that passes
//! `Mesh64::closed_manifold_check()` *by construction*, so the exact
//! volume gates measure the very polyhedron the kernel then consumes.
//!
//! Orientation rules, derived once and reused (M2 basis lesson):
//! - `Region2` stores outer loops CCW, hole loops CW — material is always
//!   on the LEFT while walking a loop.
//! - Extrude walls (sketch in XY, sweep along +z): for consecutive loop
//!   points (i, j) the quad (bi, bj, tj, ti) has normal d(i->j) x ẑ =
//!   right-of-travel = out-of-material, correct for both CCW outers and
//!   CW holes under the material-on-left contract.
//! - Caps: the region triangulation is CCW => +z-facing; the low/start cap
//!   reverses winding, the high/end cap uses it as generated.
//! - Revolve about an in-plane axis (cross-product derivation, then
//!   pinned by signed-volume > 0 + Euler gates): wall quad
//!   (ring i -> i+1) in increasing sweep order faces outward; start cap
//!   as-is, end cap reversed.
//!
//! Cap triangulation with holes: bridge every hole into the merged
//! polygon with the shortest "clear" cut (a double-directed bridge edge,
//! keeping the polygon weakly simple and CCW), then ear-clip. Internal
//! invariants checked in-module: area(merged) == area(outer) − Σ|area(hole)|
//! and Σ area(tris) == region area — a wrong-side bridge breaks them
//! immediately instead of producing sliver geometry.

use std::fmt;

use glam::DVec3;

use crate::sketch::loops::Region2;
use crate::tess::Mesh64;

/// Solidify failure with a human-readable, deterministic message.
#[derive(Clone, Debug)]
pub struct SweepError(pub String);

impl fmt::Display for SweepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for SweepError {}

type P2 = [f64; 2];

fn cross2(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn dot2(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn dv3(p: P2) -> DVec3 {
    DVec3::new(p[0], p[1], 0.0)
}

/// Signed area of a polygon (no repeated closing vertex).
fn area(pts: &[P2]) -> f64 {
    let mut s = 0.0;
    for i in 0..pts.len() {
        s += cross2(pts[i], pts[(i + 1) % pts.len()]);
    }
    s / 2.0
}

fn on_segment(p: P2, s: P2, t: P2) -> bool {
    if cross2(sub(t, s), sub(p, s)).abs() > 1e-12 {
        return false;
    }
    let len2 = dot2(sub(t, s), sub(t, s));
    let dot = dot2(sub(t, s), sub(p, s));
    dot > 1e-12 && dot < len2 - 1e-12
}

/// Do closed segments ab and uv intersect at a point strictly away from
/// the set {a, b}? (a, b are bridge ends; proper crossings, touches and
/// overlaps elsewhere all block the bridge.)
fn segment_blocks_bridge(a: P2, b: P2, u: P2, v: P2) -> bool {
    let same = |p: P2, q: P2| p[0] == q[0] && p[1] == q[1];
    let d1 = cross2(sub(b, a), sub(u, a));
    let d2 = cross2(sub(b, a), sub(v, a));
    let d3 = cross2(sub(v, u), sub(a, u));
    let d4 = cross2(sub(v, u), sub(b, u));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true; // proper crossing
    }
    // collinear / touching: endpoints of uv strictly on ab...
    for p in [u, v] {
        if same(p, a) || same(p, b) {
            continue;
        }
        if on_segment(p, a, b) {
            return true;
        }
    }
    // ...or bridge ends strictly on uv (bridge grazing an edge interior)
    for p in [a, b] {
        if on_segment(p, u, v) {
            return true;
        }
    }
    false
}

fn point_in_poly(p: P2, poly: &[P2]) -> bool {
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

/// Inside OR on boundary: conservative ear test (a vertex on an ear edge
/// rejects the ear; safe, marginally slower).
fn tri_contains(p: P2, t: [P2; 3]) -> bool {
    let [a, b, c] = t;
    let e0 = cross2(sub(b, a), sub(p, a));
    let e1 = cross2(sub(c, b), sub(p, b));
    let e2 = cross2(sub(a, c), sub(p, c));
    e0 >= -1e-12 && e1 >= -1e-12 && e2 >= -1e-12
}

/// Triangulate a region (outer CCW + holes CW). The returned vertex list
/// is the outer loop points followed by each hole's points in stored
/// order; triangle indices are positions in that concatenation, so
/// callers can reuse the SAME indices for walls and both end caps.
pub fn triangulate_region(region: &Region2) -> Result<(Vec<P2>, Vec<[u32; 3]>), SweepError> {
    if area(&region.outer) <= 0.0 {
        return Err(SweepError(format!(
            "triangulate: outer loop is not CCW (signed area {})",
            area(&region.outer)
        )));
    }
    for (hi, h) in region.holes.iter().enumerate() {
        if area(h) >= 0.0 {
            return Err(SweepError(format!(
                "triangulate: hole #{hi} is not CW (signed area {}) — region contract violated",
                area(h)
            )));
        }
    }

    // global point array + per-loop global id lists
    let mut gpts: Vec<P2> = Vec::new();
    let mut global: Vec<Vec<u32>> = Vec::new();
    for l in std::iter::once(&region.outer).chain(region.holes.iter()) {
        let mut ids = Vec::with_capacity(l.len());
        for p in l {
            gpts.push(*p);
            ids.push((gpts.len() - 1) as u32);
        }
        global.push(ids);
    }

    // every loop edge blocks candidate bridges
    let mut edges: Vec<(u32, u32)> = Vec::new();
    for ids in &global {
        let n = ids.len();
        for i in 0..n {
            edges.push((ids[i], ids[(i + 1) % n]));
        }
    }

    // merged polygon (global ids), starting from the outer loop (CCW)
    let mut merged: Vec<u32> = global[0].clone();
    for (hi, hole_ids) in global[1..].iter().enumerate() {
        let mut best: Option<(f64, u32, usize)> = None; // (len2, hole vid, merged pos)
        for &hv in hole_ids {
            for (mp, &mv) in merged.iter().enumerate() {
                let (h, o) = (gpts[hv as usize], gpts[mv as usize]);
                if h[0] == o[0] && h[1] == o[1] {
                    continue;
                }
                let mut blocked = edges
                    .iter()
                    .any(|&(u, v)| segment_blocks_bridge(h, o, gpts[u as usize], gpts[v as usize]));
                for w in merged.windows(2) {
                    if segment_blocks_bridge(h, o, gpts[w[0] as usize], gpts[w[1] as usize]) {
                        blocked = true;
                        break;
                    }
                }
                if merged.len() > 1
                    && segment_blocks_bridge(
                        h,
                        o,
                        gpts[*merged.last().unwrap() as usize],
                        gpts[merged[0] as usize],
                    )
                {
                    blocked = true;
                }
                if blocked {
                    continue;
                }
                // bridge midpoint must stay in material
                let mid = [(h[0] + o[0]) / 2.0, (h[1] + o[1]) / 2.0];
                if !point_in_poly(mid, &region.outer)
                    || region.holes.iter().any(|hl| point_in_poly(mid, hl))
                {
                    continue;
                }
                let len2 = (h[0] - o[0]).powi(2) + (h[1] - o[1]).powi(2);
                if best.is_none_or(|(bl, _, _)| len2 < bl) {
                    best = Some((len2, hv, mp));
                }
            }
        }
        let Some((_, hv, mp)) = best else {
            return Err(SweepError(format!(
                "triangulate: no clear bridge for hole #{hi} — self-intersecting or axis/touching region?"
            )));
        };
        // rotate the hole cycle to start at hv; bridge out along
        // (o_j -> hv), walk the full hole cycle back to hv, bridge back
        // along (hv -> o_j) — the same segment traversed both directions,
        // so bridge terms cancel in the shoelace sum and the merged area
        // equals outer minus holes exactly.
        let start = hole_ids.iter().position(|&g| g == hv).unwrap_or(0);
        let mut cycle: Vec<u32> = Vec::with_capacity(hole_ids.len() + 2);
        for k in 0..=hole_ids.len() {
            cycle.push(hole_ids[(start + k) % hole_ids.len()]);
        }
        let oj = merged[mp];
        cycle.push(oj);
        merged.splice(mp + 1..mp + 1, cycle);
    }

    // area invariant: the merged polygon covers exactly outer minus holes
    let want = area(&region.outer) - region.holes.iter().map(|h| area(h).abs()).sum::<f64>();
    let mpts: Vec<P2> = merged.iter().map(|&g| gpts[g as usize]).collect();
    let got = area(&mpts);
    let scale = want.abs().max(1.0);
    if (got - want).abs() > 1e-9 * scale {
        return Err(SweepError(format!(
            "triangulate: merged area {got} != outer − holes {want} — bad bridge"
        )));
    }

    // ear-clip the merged weakly-simple CCW polygon
    let mut idx = merged;
    let mut tris: Vec<[u32; 3]> = Vec::new();
    let mut guard = idx.len() * idx.len() + 16;
    while idx.len() > 3 && guard > 0 {
        guard -= 1;
        let n = idx.len();
        // bridge corner ids appear twice: clipping one of them consumes a
        // bridge edge and detaches the hole. Never clip a duplicated tip
        // unless nothing else is available (fallback pass).
        let is_dup = |id: u32| idx.iter().filter(|&x| *x == id).count() > 1;
        let mut clipped: Option<(usize, [u32; 3])> = None;
        for allow_dup in [false, true] {
            if clipped.is_some() {
                break;
            }
            'ear: for i in 0..n {
                let (a, b, c) = (idx[i], idx[(i + 1) % n], idx[(i + 2) % n]);
                if !allow_dup && (is_dup(a) || is_dup(b) || is_dup(c)) {
                    continue;
                }
                let (pa, pb, pc) = (gpts[a as usize], gpts[b as usize], gpts[c as usize]);
                let ea = cross2(sub(pb, pa), sub(pc, pb));
                if ea <= 1e-12 * scale {
                    continue; // reflex or collinear: not an ear
                }
                for &v in &idx {
                    if v == a || v == b || v == c {
                        continue;
                    }
                    let pv = gpts[v as usize];
                    // bridged duplicates of the ear corners: allowed on the tip
                    if (pv[0] == pa[0] && pv[1] == pa[1])
                        || (pv[0] == pb[0] && pv[1] == pb[1])
                        || (pv[0] == pc[0] && pv[1] == pc[1])
                    {
                        continue;
                    }
                    if tri_contains(pv, [pa, pb, pc]) {
                        continue 'ear;
                    }
                }
                clipped = Some((i, [a, b, c]));
                break;
            }
        }
        let Some((i, tri)) = clipped else {
            return Err(SweepError(
                "triangulate: no ear found — self-intersecting region".into(),
            ));
        };
        tris.push(tri);
        idx.remove((i + 1) % n); // remove ear tip
    }
    if idx.len() != 3 {
        return Err(SweepError(format!(
            "triangulate: ear clipping stalled with {} vertices",
            idx.len()
        )));
    }
    let t = [idx[0], idx[1], idx[2]];
    let ea = cross2(
        sub(gpts[t[1] as usize], gpts[t[0] as usize]),
        sub(gpts[t[2] as usize], gpts[t[1] as usize]),
    );
    tris.push(if ea < 0.0 { [t[0], t[2], t[1]] } else { t });

    // planar sanity: summed signed triangle area == region area
    let asum: f64 = tris
        .iter()
        .map(|t| {
            cross2(
                sub(gpts[t[1] as usize], gpts[t[0] as usize]),
                sub(gpts[t[2] as usize], gpts[t[0] as usize]),
            ) / 2.0
        })
        .sum();
    if (asum - want).abs() > 1e-9 * scale {
        return Err(SweepError(format!(
            "triangulate: triangle area sum {asum} != region area {want}"
        )));
    }
    Ok((gpts, tris))
}

/// Extrude a region along +z from `z0` to `z1` (sketch plane is z = 0).
pub fn extrude_region(region: &Region2, z0: f64, z1: f64) -> Result<Mesh64, SweepError> {
    if !z1.is_finite() || z1 <= z0 {
        return Err(SweepError(format!(
            "extrude: height must be > 0 and finite (got z0={z0}, z1={z1})"
        )));
    }
    let (pts, tris) = triangulate_region(region)?;
    let n = pts.len() as u32;
    let mut m = Mesh64::default();
    for p in &pts {
        m.positions.push([p[0], p[1], z0]);
    }
    for p in &pts {
        m.positions.push([p[0], p[1], z1]);
    }
    // caps: CCW tris face +z; bottom (z0) reverses winding
    for t in &tris {
        let (a, b, c) = (t[0], t[1], t[2]);
        m.indices.extend_from_slice(&[a, c, b]);
        m.indices.extend_from_slice(&[a + n, b + n, c + n]);
    }
    // walls from every loop edge (outer CCW and hole CW: left = material).
    // Loop points are globally indexed like `gpts`: outer first, then each
    // hole in stored order — walls must use the SAME global indices.
    let mut base: u32 = 0;
    let mut loops: Vec<(&Vec<P2>, u32)> = vec![(&region.outer, base)];
    base += region.outer.len() as u32;
    for h in &region.holes {
        loops.push((h, base));
        base += h.len() as u32;
    }
    for (l, lb) in loops {
        let ln = l.len() as u32;
        for k in 0..ln {
            let (i, j) = (lb + k, lb + (k + 1) % ln);
            let (ti, tj) = (i + n, j + n);
            m.indices.extend_from_slice(&[i, j, tj, i, tj, ti]);
        }
    }
    Ok(m)
}

/// An in-plane sketch axis for revolve: a point and a unit direction.
#[derive(Clone, Copy, Debug)]
pub struct Axis2 {
    pub point: P2,
    pub dir: P2,
}

impl Axis2 {
    pub fn through(a: P2, b: P2) -> Result<Axis2, SweepError> {
        let d = sub(b, a);
        let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
        if len <= 0.0 || !len.is_finite() {
            return Err(SweepError("revolve axis: zero-length direction".into()));
        }
        Ok(Axis2 {
            point: a,
            dir: [d[0] / len, d[1] / len],
        })
    }
}

/// Revolve a region about an in-plane axis through the sketch plane z = 0.
/// `angle` radians in (0, 2pi]; `segs` sweep steps (a full turn uses
/// exactly `segs` rings, matching the exact-polyhedral shell gates).
///
/// Documented limit: the profile must lie STRICTLY on one side of the
/// axis. Profiles touching/crossing the axis need degenerate-ring
/// handling (lathe solids); deferred with the timeline milestone.
pub fn revolve_region(
    region: &Region2,
    axis: Axis2,
    angle: f64,
    segs: usize,
) -> Result<Mesh64, SweepError> {
    const TAU: f64 = std::f64::consts::TAU;
    if !(angle > 0.0 && angle <= TAU) || !angle.is_finite() {
        return Err(SweepError(format!(
            "revolve: sweep angle must be in (0, 2pi] rad (got {angle})"
        )));
    }
    if segs < 3 {
        return Err(SweepError(format!(
            "revolve: sweep segments must be >= 3 (got {segs})"
        )));
    }
    let full = (angle - TAU).abs() < 1e-12;

    // axis frame in the sketch plane: d along axis, n = perp-CCW; u = side
    let [dx, dy] = axis.dir;
    let n = [-dy, dx];
    let to_uv = |p: P2| -> P2 {
        let q = sub(p, axis.point);
        [q[0] * n[0] + q[1] * n[1], q[0] * dx + q[1] * dy]
    };

    let mut loops: Vec<Vec<P2>> = Vec::new();
    let mut min_u = f64::INFINITY;
    let mut max_u = f64::NEG_INFINITY;
    let mut vspan = (f64::INFINITY, f64::NEG_INFINITY);
    for l in std::iter::once(&region.outer).chain(region.holes.iter()) {
        let uv: Vec<P2> = l.iter().map(|p| to_uv(*p)).collect();
        for p in &uv {
            min_u = min_u.min(p[0]);
            max_u = max_u.max(p[0]);
            vspan = (vspan.0.min(p[1]), vspan.1.max(p[1]));
        }
        loops.push(uv);
    }
    let diag = ((max_u - min_u).powi(2) + (vspan.1 - vspan.0).powi(2)).sqrt();
    let touch_tol = 1e-9 * diag.max(1e-12);
    if min_u < -touch_tol && max_u > touch_tol {
        return Err(SweepError(format!(
            "revolve: region straddles the axis (signed distance in [{min_u:.6}, {max_u:.6}]) — the axis must not cross the profile"
        )));
    }
    // to_uv is a reflection (n x d = -1 always), so loop orientation flips
    // relative to the sketch plane; and a profile on the -u side is folded
    // to +u with the sweep origin rotated by pi (a partial revolve of a
    // negative-side profile sweeps the OPPOSITE wedge — pi keeps the
    // solid in the right place). Orientation is then normalized directly
    // by signed area in the (u,v) plane: outer CCW, holes CW.
    let on_neg = max_u <= 0.0;
    if on_neg {
        for l in &mut loops {
            for p in l.iter_mut() {
                p[0] = -p[0];
            }
        }
    }
    let umin = if on_neg { -max_u } else { min_u };
    if umin <= touch_tol {
        return Err(SweepError(format!(
            "revolve: profile touches the axis (min distance {umin:.3e}) — axis must be external (limit)"
        )));
    }
    if area(&loops[0]) < 0.0 {
        loops[0].reverse();
    }
    for l in loops[1..].iter_mut() {
        if area(l) > 0.0 {
            l.reverse();
        }
    }
    let alpha0 = if on_neg { std::f64::consts::PI } else { 0.0 };

    // 3D embedding: point(u, v, alpha) = O + v*d + u*(cos(a)*n + sin(a)*z)
    let d3 = DVec3::new(dx, dy, 0.0);
    let n3 = dv3(n);
    let o3 = dv3(axis.point);
    let z3 = DVec3::Z;
    let rings = if full { segs } else { segs + 1 };
    let pos = |i: usize, uv: P2| -> [f64; 3] {
        let a = alpha0 + angle * i as f64 / segs as f64;
        let rhat = n3 * a.cos() + z3 * a.sin();
        (o3 + d3 * uv[1] + rhat * uv[0]).to_array()
    };

    let mut m = Mesh64::default();
    let mut starts: Vec<u32> = Vec::new();
    let mut counts: Vec<usize> = Vec::new();
    for l in &loops {
        starts.push(m.positions.len() as u32);
        counts.push(l.len());
        for i in 0..rings {
            for uv in l {
                m.positions.push(pos(i, *uv));
            }
        }
    }
    let vof = |loop_i: usize, ring: usize, pt: usize| -> u32 {
        starts[loop_i] + (ring * counts[loop_i] + pt) as u32
    };
    // walls: quad corners a=(j,i) b=(j2,i) c=(j2,i+1) e=(j,i+1). With
    // loops CCW-outer/CW-hole in the LEFT-handed (u,v) basis (n x d = -1),
    // dP/dt x dP/dalpha = u(u' d3 - v' rhat) = INWARD, so the outward
    // triangulation is (a,c,b),(a,e,c). Pinned by signed-volume gates.
    for (li, &ln) in counts.iter().enumerate() {
        for k in 0..ln {
            let (j, j2) = (k, (k + 1) % ln);
            for i in 0..segs {
                let i2 = if full { (i + 1) % segs } else { i + 1 };
                let (a, b) = (vof(li, i, j), vof(li, i, j2));
                let (c, e) = (vof(li, i2, j2), vof(li, i2, j));
                m.indices.extend_from_slice(&[a, c, b, a, e, c]);
            }
        }
    }
    if !full {
        // end caps: triangulate the UV region; its vertex order matches the
        // concatenated loop points, so cap indices reuse the ring grid.
        let uv_region = Region2 {
            outer: loops[0].clone(),
            holes: loops[1..].to_vec(),
        };
        let (_, tris) = triangulate_region(&uv_region)?;
        // global pt index -> (loop, point-in-loop)
        let locate = |g: u32| -> (usize, usize) {
            let mut acc = 0u32;
            for (li, &cnt) in counts.iter().enumerate() {
                if g < acc + cnt as u32 {
                    return (li, (g - acc) as usize);
                }
                acc += cnt as u32;
            }
            (0, 0)
        };
        for t in &tris {
            let v = t.map(locate);
            let (a0, a1) = (vof(v[0].0, 0, v[0].1), vof(v[1].0, 0, v[1].1));
            let (b0, b1) = (vof(v[2].0, 0, v[2].1), vof(v[0].0, segs, v[0].1));
            let (b2, b3) = (vof(v[1].0, segs, v[1].1), vof(v[2].0, segs, v[2].1));
            m.indices.extend_from_slice(&[a0, a1, b0]); // start cap (as-is)
            m.indices.extend_from_slice(&[b1, b3, b2]); // end cap (reversed)
        }
    }
    Ok(m)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reg(outer: &[P2], holes: &[Vec<P2>]) -> Region2 {
        let mut outer = outer.to_vec();
        if area(&outer) < 0.0 {
            outer.reverse();
        }
        let mut hs = Vec::new();
        for h in holes {
            let mut h = h.clone();
            if area(&h) > 0.0 {
                h.reverse(); // holes must be CW
            }
            hs.push(h);
        }
        Region2 { outer, holes: hs }
    }

    const SQ: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];

    #[test]
    fn triangulate_square_area_exact() {
        let (pts, tris) = triangulate_region(&reg(&SQ, &[])).unwrap();
        assert_eq!(tris.len(), 2);
        let asum: f64 = tris
            .iter()
            .map(|t| {
                cross2(
                    sub(pts[t[1] as usize], pts[t[0] as usize]),
                    sub(pts[t[2] as usize], pts[t[0] as usize]),
                ) / 2.0
            })
            .sum();
        assert!((asum - 100.0).abs() < 1e-12);
    }

    #[test]
    fn triangulate_annulus_sums_to_region_area() {
        let hole: Vec<P2> = (0..16)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 16.0;
                [5.0 + 3.0 * a.cos(), 5.0 + 3.0 * a.sin()]
            })
            .collect();
        let hole_area = area(&hole).abs(); // 16-gon, CW-stored -> abs
        let r = reg(&SQ, &[hole]);
        let (pts, tris) = triangulate_region(&r).unwrap();
        let want = 100.0 - hole_area;
        let asum: f64 = tris
            .iter()
            .map(|t| {
                cross2(
                    sub(pts[t[1] as usize], pts[t[0] as usize]),
                    sub(pts[t[2] as usize], pts[t[0] as usize]),
                ) / 2.0
            })
            .sum();
        assert!(
            (asum - want).abs() < 1e-9,
            "sum {asum} want {want} over {} tris",
            tris.len()
        );
    }

    #[test]
    fn concave_l_shape_clips_without_ear_failure() {
        let l = [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 4.0],
            [4.0, 4.0],
            [4.0, 10.0],
            [0.0, 10.0],
        ];
        let (pts, tris) = triangulate_region(&reg(&l, &[])).unwrap();
        let asum: f64 = tris
            .iter()
            .map(|t| {
                cross2(
                    sub(pts[t[1] as usize], pts[t[0] as usize]),
                    sub(pts[t[2] as usize], pts[t[0] as usize]),
                ) / 2.0
            })
            .sum();
        assert!((asum - 64.0).abs() < 1e-9, "L area sum {asum}");
    }

    #[test]
    fn extrude_square_is_closed_exact_prism() {
        let m = extrude_region(&reg(&SQ, &[]), 0.0, 6.0).unwrap();
        let (v, e, f) = m.closed_manifold_check().unwrap();
        assert_eq!((v, e, f), (8, 18, 12));
        assert_eq!(v as i64 - e as i64 + f as i64, 2);
        assert!((m.signed_volume() - 600.0).abs() < 1e-9);
    }

    #[test]
    fn extrude_annulus_is_closed_genus_one() {
        let hole: Vec<P2> = (0..24)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 24.0;
                [5.0 + 3.0 * a.cos(), 5.0 + 3.0 * a.sin()]
            })
            .collect();
        let m = extrude_region(&reg(&SQ, &[hole]), 0.0, 5.0).unwrap();
        let (v, e, f) = m.closed_manifold_check().unwrap();
        assert_eq!(
            v as i64 - e as i64 + f as i64,
            0,
            "plate with through-hole: chi = 0"
        );
        let hole_area = 0.5 * 24.0 * 9.0 * (std::f64::consts::TAU / 24.0).sin();
        let want = (100.0 - hole_area) * 5.0;
        assert!((m.signed_volume() - want).abs() < 1e-9, "want {want}");
    }

    #[test]
    fn revolve_square_full_turn_is_closed_shell_and_positive() {
        // square x in [10, 12], y in [0, 4]; axis = y axis at x = 0
        let sq = [[10.0, 0.0], [12.0, 0.0], [12.0, 4.0], [10.0, 4.0]];
        let m = revolve_region(
            &reg(&sq, &[]),
            Axis2::through([0.0, 0.0], [0.0, 1.0]).unwrap(),
            std::f64::consts::TAU,
            48,
        )
        .unwrap();
        let (v, e, f) = m.closed_manifold_check().unwrap();
        assert_eq!(
            v as i64 - e as i64 + f as i64,
            0,
            "full revolve of simple region: chi 0 (torus surface)"
        );
        assert!(m.signed_volume() > 0.0, "winding must face outward");
    }

    #[test]
    fn revolve_straddling_axis_rejected() {
        let sq = [[-2.0, 0.0], [3.0, 0.0], [3.0, 4.0], [-2.0, 4.0]];
        let e = revolve_region(
            &reg(&sq, &[]),
            Axis2::through([0.0, 0.0], [0.0, 1.0]).unwrap(),
            std::f64::consts::TAU,
            16,
        )
        .unwrap_err();
        assert!(e.to_string().contains("straddles"), "{e}");
    }

    #[test]
    fn revolve_touching_axis_rejected_with_named_limit() {
        let sq = [[0.0, 0.0], [3.0, 0.0], [3.0, 4.0], [0.0, 4.0]];
        let e = revolve_region(
            &reg(&sq, &[]),
            Axis2::through([0.0, 0.0], [0.0, 1.0]).unwrap(),
            std::f64::consts::TAU,
            16,
        )
        .unwrap_err();
        assert!(e.to_string().contains("touches the axis"), "{e}");
    }
}
