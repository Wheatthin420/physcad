//! M6: stable topology naming — face identity across feature recomputation.
//!
//! The problem (docs/SURVEY.md §2): mesh/CAD kernels re-derive face
//! indices from traversal order, so any rebuild can silently break
//! downstream references. Our answer, per the survey's mandate, is to
//! carry identity as *explicit creation-history data*: every feature
//! declares the analytic surfaces its faces live on, keyed by
//! `FeatureId`; a rebuilt mesh is then **re-identified** against those
//! declarations rather than trusted to keep index order.
//!
//! The algorithm (full text + failure modes: docs/NAMING.md):
//!
//! 1. Each leaf feature publishes a small table of `FaceRecord`s —
//!    (feature id, structural tag, analytic `Surface`). Tags come from
//!    the feature's *construction semantics* (box face `face:+x`,
//!    extrude side `wall:<sketch-entity>`, cylinder `wall:cyl`), never
//!    from measured geometry or triangle order.
//! 2. `Move` translates the operand's surfaces; union/cut/intersect
//!    *merge* the two operand tables (first operand wins ties). A cut
//!    creates no names of its own: new wall faces lie on the tool's
//!    recorded surfaces and inherit the tool feature's name — exactly
//!    the CAD semantic ("the bore wall belongs to the bore feature").
//! 3. Every output triangle is classified by testing all three vertices
//!    against the root table within `MATCH_EPS`; faces are then grouped
//!    by (key, mesh-connected component), so a name split into islands
//!    stays one logical face with deterministic component indices.
//!
//! This is the *mesh-based continuity heuristic* the milestone allows:
//! no B-rep kernel, no reliance on manifold keeping triangle order or
//! provenance across booleans (it makes no such promise). Names survive
//! parameter edits because the surface tables are *recomputed from the
//! same typed feature data* — the identity is in the data model, the
//! geometry only has to land on it.

use std::collections::BTreeMap;

use glam::DVec3;

use crate::sketch::loops::Region2;
use crate::sketch::{Entity, Sketch};
use crate::sweep::Axis2;
use crate::timeline::FeatureId;

/// Vertex-to-surface tolerance (mm) for classifying kernel output
/// triangles. f64 kernel + manifold intersection math stay ~1e-12 off
/// the declaring surface; 1e-7 is 5 orders of magnitude of headroom and
/// still 8 orders below any real feature dimension.
pub const MATCH_EPS: f64 = 1e-7;

/// An analytic surface family a named face lives on. Plane normals are
/// canonicalized (largest-magnitude component positive) so identical
/// planes compare and display identically.
#[derive(Clone, Debug, PartialEq)]
pub enum Surface {
    Plane {
        n: [f64; 3],
        off: f64,
    },
    Cylinder {
        p0: [f64; 3],
        d: [f64; 3],
        r: f64,
    },
    Sphere {
        c: [f64; 3],
        r: f64,
    },
    Cone {
        apex: [f64; 3],
        d: [f64; 3],
        cos_a: f64,
    },
    Torus {
        c: [f64; 3],
        d: [f64; 3],
        big: f64,
        small: f64,
    },
}

fn canon_plane(n: DVec3, off: f64) -> Surface {
    // Normalize FIRST, scaling `off` with the same factor, so the plane
    // equation n.p = off keeps describing the same set of points.
    let len = n.length();
    let (n, off) = if len > 0.0 {
        (n / len, off / len)
    } else {
        (n, off)
    };
    let flip = match (n.x, n.y, n.z) {
        (x, _, _) if x < 0.0 => true,
        (x, _, _) if x > 0.0 => false,
        (_, y, _) if y < 0.0 => true,
        (_, y, _) if y > 0.0 => false,
        (_, _, z) => z < 0.0,
    };
    if flip {
        Surface::Plane {
            n: (-n).to_array(),
            off: -off,
        }
    } else {
        Surface::Plane {
            n: n.to_array(),
            off,
        }
    }
}

impl Surface {
    /// Plane through `p` with normal `n` (normalized, canonicalized).
    pub fn plane(n: DVec3, p: DVec3) -> Surface {
        let off = n.dot(p);
        canon_plane(n, off)
    }

    pub fn translate(&self, t: DVec3) -> Surface {
        match self.clone() {
            Surface::Plane { n, off } => Surface::Plane {
                n,
                off: off + DVec3::from_array(n).dot(t),
            },
            Surface::Cylinder { p0, d, r } => Surface::Cylinder {
                p0: (DVec3::from_array(p0) + t).to_array(),
                d,
                r,
            },
            Surface::Sphere { c, r } => Surface::Sphere {
                c: (DVec3::from_array(c) + t).to_array(),
                r,
            },
            Surface::Cone { apex, d, cos_a } => Surface::Cone {
                apex: (DVec3::from_array(apex) + t).to_array(),
                d,
                cos_a,
            },
            Surface::Torus { c, d, big, small } => Surface::Torus {
                c: (DVec3::from_array(c) + t).to_array(),
                d,
                big,
                small,
            },
        }
    }

    /// Distance from `p` to this surface (0 = on it).
    pub fn distance(&self, p: DVec3) -> f64 {
        match *self {
            Surface::Plane { n, off } => (DVec3::from_array(n).dot(p) - off).abs(),
            Surface::Cylinder { p0, d, r } => {
                let d = DVec3::from_array(d);
                let q = p - DVec3::from_array(p0);
                let rho2 = q.length_squared() - q.dot(d).powi(2);
                let rho = if rho2 > 0.0 { rho2.sqrt() } else { 0.0 };
                (rho - r).abs()
            }
            Surface::Sphere { c, r } => ((p - DVec3::from_array(c)).length() - r).abs(),
            Surface::Cone { apex, d, cos_a } => {
                let q = p - DVec3::from_array(apex);
                let l = q.length();
                if l < 1e-12 {
                    return 0.0; // apex: on every cone by definition
                }
                let cos_q = (q.dot(DVec3::from_array(d)) / l).abs().min(1.0);
                let sin_q = (1.0 - cos_q * cos_q).max(0.0).sqrt();
                let sin_a = (1.0 - cos_a * cos_a).max(0.0).sqrt();
                l * (cos_q * sin_a - sin_q * cos_a).abs()
            }
            Surface::Torus { c, d, big, small } => {
                let d = DVec3::from_array(d);
                let q = p - DVec3::from_array(c);
                let h = q.dot(d);
                let rho = (q - d * h).length();
                ((h * h + (rho - big) * (rho - big)).sqrt() - small).abs()
            }
        }
    }
}

/// One declared face of one feature: identity + where it lives.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRecord {
    pub feature: FeatureId,
    pub tag: String,
    pub surface: Surface,
}

impl FaceRecord {
    pub fn new(feature: FeatureId, tag: &str, surface: Surface) -> FaceRecord {
        FaceRecord {
            feature,
            tag: tag.to_string(),
            surface,
        }
    }

    /// Stable key: creation-history identity, independent of user names.
    pub fn key(&self) -> String {
        format!("F{}:{}", self.feature.0, self.tag)
    }
}

/// Box centered on origin (matches `tess::box64`): planes x=±w/2 etc.
/// Canonical normal is always +axis, so the `off` locates the slab side.
pub fn records_box(id: FeatureId, w: f64, h: f64, d: f64) -> Vec<FaceRecord> {
    let (hw, hh, hd) = (w / 2.0, h / 2.0, d / 2.0);
    vec![
        FaceRecord::new(id, "face:-x", canon_plane(-DVec3::X, hw)),
        FaceRecord::new(id, "face:+x", canon_plane(DVec3::X, hw)),
        FaceRecord::new(id, "face:-y", canon_plane(-DVec3::Y, hh)),
        FaceRecord::new(id, "face:+y", canon_plane(DVec3::Y, hh)),
        FaceRecord::new(id, "face:-z", canon_plane(-DVec3::Z, hd)),
        FaceRecord::new(id, "face:+z", canon_plane(DVec3::Z, hd)),
    ]
}

/// Cylinder around +Y, centered on origin (matches `tess::cylinder64`).
pub fn records_cylinder(id: FeatureId, r: f64, h: f64) -> Vec<FaceRecord> {
    let hh = h / 2.0;
    vec![
        FaceRecord::new(
            id,
            "wall:cyl",
            Surface::Cylinder {
                p0: [0.0, 0.0, 0.0],
                d: [0.0, 1.0, 0.0],
                r,
            },
        ),
        FaceRecord::new(id, "cap:-y", canon_plane(-DVec3::Y, hh)),
        FaceRecord::new(id, "cap:+y", canon_plane(DVec3::Y, hh)),
    ]
}

pub fn records_sphere(id: FeatureId, r: f64) -> Vec<FaceRecord> {
    vec![FaceRecord::new(
        id,
        "surface",
        Surface::Sphere {
            c: [0.0, 0.0, 0.0],
            r,
        },
    )]
}

pub fn records_torus(id: FeatureId, r: f64, tube: f64) -> Vec<FaceRecord> {
    vec![FaceRecord::new(
        id,
        "surface",
        Surface::Torus {
            c: [0.0, 0.0, 0.0],
            d: [0.0, 1.0, 0.0],
            big: r,
            small: tube,
        },
    )]
}

/// Coordinates of a point entity (kind checked at sketch add time).
fn xy(sk: &Sketch, i: usize) -> Option<DVec3> {
    match sk.entities.get(i).map(|(_, e)| e) {
        Some(Entity::Point { x, y }) => Some(DVec3::new(*x, *y, 0.0)),
        _ => None,
    }
}

fn arc_radius(sk: &Sketch, center: usize, start: usize) -> Option<f64> {
    Some((xy(sk, start)? - xy(sk, center)?).length())
}

/// Extrude leaf: caps at z=offset / offset+height, one wall face per
/// curve entity (line -> plane through the line + z; circle/arc ->
/// cylinder along +z at its center/radius). Declared for ALL curve
/// entities of the swept sketch: entities that end up interior declare
/// a surface no triangle matches (harmless); coincident duplicate
/// entities tie-break by file order (failure mode FM-4).
pub fn records_extrude(id: FeatureId, solved: &Sketch, z0: f64, z1: f64) -> Vec<FaceRecord> {
    let mut out = vec![
        FaceRecord::new(id, "cap:bottom", canon_plane(DVec3::Z, z0)),
        FaceRecord::new(id, "cap:top", canon_plane(DVec3::Z, z1)),
    ];
    for (name, e) in &solved.entities {
        match *e {
            Entity::Point { .. } => {}
            Entity::Line { a, b } => {
                let (pa, pb) = match (xy(solved, a), xy(solved, b)) {
                    (Some(p), Some(q)) => (p, q),
                    _ => continue,
                };
                let dir = pb - pa;
                if dir.length() < 1e-12 {
                    continue; // degenerate; region extraction rejects it
                }
                // Normal in-plane, perpendicular to the edge: (dir x z).
                let n = dir.cross(DVec3::Z);
                out.push(FaceRecord::new(
                    id,
                    &format!("wall:{name}"),
                    Surface::plane(n, pa),
                ));
            }
            Entity::Circle { center, radius } => {
                let c = match xy(solved, center) {
                    Some(c) => c,
                    None => continue,
                };
                out.push(FaceRecord::new(
                    id,
                    &format!("wall:{name}"),
                    Surface::Cylinder {
                        p0: c.to_array(),
                        d: [0.0, 0.0, 1.0],
                        r: radius,
                    },
                ));
            }
            Entity::Arc { center, start, .. } => {
                let c = match xy(solved, center) {
                    Some(c) => c,
                    None => continue,
                };
                let r = match arc_radius(solved, center, start) {
                    Some(r) => r,
                    None => continue,
                };
                out.push(FaceRecord::new(
                    id,
                    &format!("wall:{name}"),
                    Surface::Cylinder {
                        p0: c.to_array(),
                        d: [0.0, 0.0, 1.0],
                        r,
                    },
                ));
            }
        }
    }
    out
}

/// Revolve leaf: one wall face per profile curve entity (about the
/// revolve axis: line parallel -> cylinder, line perpendicular -> plane
/// normal to axis, slanted line -> cone; circle/arc -> torus, or sphere
/// if its center sits on the axis) plus, for partial sweeps, the two
/// cap planes containing the axis. `axis` is the 2D sketch frame from
/// the axis line entity; `regs` are the same regions the sweep used, so
/// the negative-side fold decision (`alpha0 = pi`) matches the mesh
/// builder bit-for-bit.
pub fn records_revolve(
    id: FeatureId,
    solved: &Sketch,
    axis: Axis2,
    angle: f64,
    axis_entity: &str,
    regs: &[Region2],
) -> Vec<FaceRecord> {
    const TAU: f64 = std::f64::consts::TAU;
    let [dx, dy] = axis.dir;
    let n2 = [-dy, dx]; // sketch-plane radial
    let o3 = DVec3::new(axis.point[0], axis.point[1], 0.0);
    let d3 = DVec3::new(dx, dy, 0.0);
    let n3 = DVec3::new(n2[0], n2[1], 0.0);
    let to_uv = |p: DVec3| -> [f64; 2] {
        let q = [p.x - axis.point[0], p.y - axis.point[1]];
        [q[0] * n2[0] + q[1] * n2[1], q[0] * dx + q[1] * dy]
    };

    // Fold decision, exactly as `sweep::revolve_region`: profile strictly
    // on the -u side is mirrored and the sweep starts at pi.
    let mut max_u = f64::NEG_INFINITY;
    for r in regs {
        for l in std::iter::once(&r.outer).chain(r.holes.iter()) {
            for p in l {
                max_u = max_u.max(to_uv(DVec3::new(p[0], p[1], 0.0))[0]);
            }
        }
    }
    let on_neg = max_u <= 0.0;

    let mut out = Vec::new();
    for (name, e) in &solved.entities {
        if name == axis_entity {
            continue; // construction entity, never a face
        }
        let (u0, v0, u1, v1) = match *e {
            Entity::Line { a, b } => {
                let (pa, pb) = match (xy(solved, a), xy(solved, b)) {
                    (Some(p), Some(q)) => (p, q),
                    _ => continue,
                };
                let (t0, t1) = (to_uv(pa), to_uv(pb));
                (t0[0], t0[1], t1[0], t1[1])
            }
            Entity::Circle { center, radius } => {
                let c = match xy(solved, center) {
                    Some(c) => c,
                    None => continue,
                };
                let t = to_uv(c);
                let tag = format!("wall:{name}");
                let big = t[0].abs();
                let surf = if big > MATCH_EPS {
                    Surface::Torus {
                        c: (o3 + d3 * t[1]).to_array(),
                        d: d3.to_array(),
                        big,
                        small: radius,
                    }
                } else {
                    Surface::Sphere {
                        c: (o3 + d3 * t[1]).to_array(),
                        r: radius,
                    }
                };
                out.push(FaceRecord::new(id, &tag, surf));
                continue;
            }
            Entity::Arc { center, start, .. } => {
                let c = match xy(solved, center) {
                    Some(c) => c,
                    None => continue,
                };
                let radius = match arc_radius(solved, center, start) {
                    Some(r) => r,
                    None => continue,
                };
                let t = to_uv(c);
                let tag = format!("wall:{name}");
                let big = t[0].abs();
                let surf = if big > MATCH_EPS {
                    Surface::Torus {
                        c: (o3 + d3 * t[1]).to_array(),
                        d: d3.to_array(),
                        big,
                        small: radius,
                    }
                } else {
                    Surface::Sphere {
                        c: (o3 + d3 * t[1]).to_array(),
                        r: radius,
                    }
                };
                out.push(FaceRecord::new(id, &tag, surf));
                continue;
            }
            Entity::Point { .. } => continue,
        };
        // Line profile edge -> cylinder / plane / cone about the axis.
        let (u0, u1) = (u0.abs(), u1.abs());
        let len = ((u1 - u0).powi(2) + (v1 - v0).powi(2)).sqrt();
        if len < 1e-12 {
            continue;
        }
        let tol = 1e-9 * len.max(1.0);
        let tag = format!("wall:{name}");
        if (u1 - u0).abs() <= tol {
            out.push(FaceRecord::new(
                id,
                &tag,
                Surface::Cylinder {
                    p0: (o3 + d3 * v0).to_array(),
                    d: d3.to_array(),
                    r: u0,
                },
            ));
        } else if (v1 - v0).abs() <= tol {
            // perpendicular edge sweeps a disc perpendicular to the axis
            out.push(FaceRecord::new(id, &tag, canon_plane(d3, d3.dot(o3) + v0)));
        } else {
            // slanted edge: apex is where the edge line crosses u = 0
            let v_a = v0 - u0 * (v1 - v0) / (u1 - u0);
            let e = [u1 - u0, v1 - v0];
            let cos_a = e[1].abs() / len;
            out.push(FaceRecord::new(
                id,
                &tag,
                Surface::Cone {
                    apex: (o3 + d3 * v_a).to_array(),
                    d: d3.to_array(),
                    cos_a,
                },
            ));
        }
    }

    // Partial sweeps: start/end cap planes contain the axis; their
    // normals rotate with the sweep angle exactly as the mesh rings do.
    if (angle - TAU).abs() >= 1e-12 {
        let alpha0 = if on_neg { std::f64::consts::PI } else { 0.0 };
        let rhat = |a: f64| n3 * a.cos() + DVec3::Z * a.sin();
        let cap = |a: f64| Surface::plane(d3.cross(rhat(a)), o3);
        out.push(FaceRecord::new(id, "cap:start", cap(alpha0)));
        out.push(FaceRecord::new(id, "cap:end", cap(alpha0 + angle)));
    }
    out
}

/// The root table a rebuilt solid is re-identified against: a flat list
/// of declaring records, first match wins (deterministic tie-break;
/// operand order A-then-B is baked in by `csg::evaluate_with`).
pub type FaceTable = Vec<FaceRecord>;

/// True if every triangle vertex lies on the surface.
fn tri_matches(tri: &[DVec3; 3], s: &Surface) -> bool {
    tri.iter().all(|p| s.distance(*p) <= MATCH_EPS)
}

/// A named face of the evaluated solid: key + measure.
#[derive(Clone, Debug, PartialEq)]
pub struct NamedFace {
    /// `F{feature}:{tag}`, plus `#{k}` when the logical face resolved to
    /// more than one mesh-connected component (sorted by centroid).
    pub key: String,
    pub feature: FeatureId,
    pub tag: String,
    pub component: usize,
    pub num_tris: usize,
    pub area_mm2: f64,
    pub centroid: [f64; 3],
}

/// Classify every kernel triangle against the root table, group by
/// (record, connected component), and emit named faces sorted by
/// (feature, tag, component). Unmatched triangles never vanish and never
/// borrow a neighbor's name: they collect under `F0:unmatched` so
/// callers (and tests) see naming failures loudly.
pub fn name_faces(verts: &[DVec3], tris: &[[u32; 3]], table: &FaceTable) -> Vec<NamedFace> {
    // tri index -> chosen record key (first match in table order)
    let mut groups: BTreeMap<(u32, String), Vec<usize>> = BTreeMap::new();
    for (ti, t) in tris.iter().enumerate() {
        let tri = [
            verts[t[0] as usize],
            verts[t[1] as usize],
            verts[t[2] as usize],
        ];
        let hit = table
            .iter()
            .find(|r| tri_matches(&tri, &r.surface))
            .map(|r| (r.feature.0, r.key()))
            .unwrap_or((
                FeatureId::UNKNOWN.0,
                format!("F{}:unmatched", FeatureId::UNKNOWN.0),
            ));
        groups.entry(hit).or_default().push(ti);
    }

    let mut out: Vec<NamedFace> = Vec::new();
    for ((fid, key), tlist) in groups {
        // split into mesh-connected components inside this key group
        let mut parent: Vec<usize> = (0..tlist.len()).collect();
        fn find(p: &[usize], mut i: usize) -> usize {
            while p[i] != i {
                i = p[i];
            }
            i
        }
        // weld map: vertex position bits -> first tri-in-group using it
        let mut weld: BTreeMap<[u64; 3], usize> = BTreeMap::new();
        for (gi, &ti) in tlist.iter().enumerate() {
            for &v in &tris[ti] {
                let keyv = verts[v as usize].to_array().map(f64::to_bits);
                if let Some(&other) = weld.get(&keyv) {
                    let (ra, rb) = (find(&parent, gi), find(&parent, other));
                    if ra != rb {
                        parent[ra] = rb;
                    }
                } else {
                    weld.insert(keyv, gi);
                }
            }
        }
        let mut comp: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for gi in 0..tlist.len() {
            comp.entry(find(&parent, gi)).or_default().push(gi);
        }
        // measure components
        let mut measured: Vec<(usize, usize, f64, [f64; 3])> = Vec::new(); // (root, tris, area, centroid)
        for members in comp.values() {
            let (mut area, mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0, 0.0);
            for &gi in members {
                let t = tris[tlist[gi]];
                let (a, b, c) = (
                    verts[t[0] as usize],
                    verts[t[1] as usize],
                    verts[t[2] as usize],
                );
                let ar = (b - a).cross(c - a).length() / 2.0;
                let tc = (a + b + c) / 3.0;
                area += ar;
                cx += tc.x * ar;
                cy += tc.y * ar;
                cz += tc.z * ar;
            }
            let inv = if area > 0.0 { 1.0 / area } else { 0.0 };
            measured.push((
                members[0],
                members.len(),
                area,
                [cx * inv, cy * inv, cz * inv],
            ));
        }
        measured.sort_by(|a, b| {
            a.3[0]
                .total_cmp(&b.3[0])
                .then(a.3[1].total_cmp(&b.3[1]))
                .then(a.3[2].total_cmp(&b.3[2]))
        });
        for (ci, (_root, ntris, area, centroid)) in measured.iter().enumerate() {
            out.push(NamedFace {
                key: if measured.len() == 1 {
                    key.clone()
                } else {
                    format!("{key}#{ci}")
                },
                feature: FeatureId(fid),
                tag: key[(key.find(':').unwrap_or(key.len()) + 1)..].to_string(),
                component: ci,
                num_tris: *ntris,
                area_mm2: *area,
                centroid: *centroid,
            });
        }
    }
    out.sort_by(|a, b| {
        a.feature
            .0
            .cmp(&b.feature.0)
            .then_with(|| a.tag.cmp(&b.tag))
            .then_with(|| a.component.cmp(&b.component))
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64, z: f64) -> DVec3 {
        DVec3::new(x, y, z)
    }

    #[test]
    fn plane_canonicalization_is_sign_stable() {
        let a = canon_plane(-DVec3::Y, -6.0);
        let b = canon_plane(DVec3::Y, 6.0);
        assert_eq!(a, b);
        // distance from y = 0 to plane y = 6 is 6 either way
        assert!((a.distance(p(0.0, 0.0, 0.0)) - 6.0).abs() < 1e-12);
    }

    #[test]
    fn surface_distances_on_and_off() {
        let cyl = Surface::Cylinder {
            p0: [0.0, 0.0, 0.0],
            d: [0.0, 1.0, 0.0],
            r: 5.0,
        };
        assert!(cyl.distance(p(5.0, 3.0, 0.0)).abs() < 1e-12);
        assert!((cyl.distance(p(6.0, 3.0, 0.0)) - 1.0).abs() < 1e-12);
        let sph = Surface::Sphere {
            c: [0.0, 0.0, 0.0],
            r: 2.0,
        };
        assert!(sph.distance(p(0.0, -2.0, 0.0)).abs() < 1e-12);
        // cone: half-angle 45deg about +Y; point at 45deg on the cone
        let cone = Surface::Cone {
            apex: [0.0, 0.0, 0.0],
            d: [0.0, 1.0, 0.0],
            cos_a: std::f64::consts::FRAC_1_SQRT_2,
        };
        assert!(cone.distance(p(3.0, 3.0, 0.0)).abs() < 1e-9);
        // perpendicular distance to the 45deg cone surface, not radial: 1/sqrt2
        assert!((cone.distance(p(4.0, 3.0, 0.0)) - 1.0 / 2.0f64.sqrt()).abs() < 1e-9);
        // torus (R=10, r=2) about +Y: outer equator point
        let tor = Surface::Torus {
            c: [0.0, 0.0, 0.0],
            d: [0.0, 1.0, 0.0],
            big: 10.0,
            small: 2.0,
        };
        assert!(tor.distance(p(12.0, 0.0, 0.0)).abs() < 1e-12);
        assert!(tor.distance(p(10.0, 2.0, 0.0)).abs() < 1e-12);
        assert!((tor.distance(p(11.0, 0.0, 0.0)) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn translate_moves_every_family() {
        let t = p(1.0, 2.0, 3.0);
        let pl = Surface::plane(DVec3::Z, p(0.0, 0.0, 5.0)).translate(t);
        assert!((pl.distance(p(0.0, 0.0, 8.0))).abs() < 1e-12);
        let cy = Surface::Cylinder {
            p0: [0.0, 0.0, 0.0],
            d: [0.0, 1.0, 0.0],
            r: 1.0,
        }
        .translate(t);
        assert!(cy.distance(p(2.0, 9.0, 3.0)).abs() < 1e-12);
        let to = Surface::Torus {
            c: [0.0, 0.0, 0.0],
            d: [0.0, 1.0, 0.0],
            big: 10.0,
            small: 2.0,
        }
        .translate(t);
        assert!(to.distance(p(13.0, 2.0, 3.0)).abs() < 1e-12);
        assert!(to.distance(p(11.0, 4.0, 3.0)).abs() < 1e-12);
    }

    #[test]
    fn box_records_match_tess_faces() {
        // box(10,4,2): faces at x=±5, y=±2, z=±1 (tess::box64 placement)
        let rs = records_box(FeatureId(1), 10.0, 4.0, 2.0);
        assert_eq!(rs.len(), 6);
        let hit = |pt: DVec3| -> String {
            rs.iter()
                .find(|r| r.surface.distance(pt) <= MATCH_EPS)
                .map(|r| r.key())
                .expect("box face must match")
        };
        assert_eq!(hit(p(5.0, 0.3, -0.2)), "F1:face:+x");
        assert_eq!(hit(p(-5.0, 1.0, 1.0)), "F1:face:-x");
        assert_eq!(hit(p(0.0, -2.0, 0.0)), "F1:face:-y");
        assert_eq!(hit(p(0.0, 0.0, 1.0)), "F1:face:+z");
        // an interior point matches nothing
        assert!(
            rs.iter()
                .all(|r| r.surface.distance(p(0.0, 0.0, 0.0)) > MATCH_EPS)
        );
    }

    #[test]
    fn cylinder_records_match_wall_and_caps() {
        let rs = records_cylinder(FeatureId(2), 5.0, 12.0);
        assert_eq!(rs[0].key(), "F2:wall:cyl");
        assert!(rs[0].surface.distance(p(0.0, 3.0, 5.0)) < 1e-12);
        assert!(rs[1].surface.distance(p(3.0, -6.0, 0.0)) < 1e-12);
        assert!(rs[2].surface.distance(p(3.0, 6.0, 0.0)) < 1e-12);
    }
}
