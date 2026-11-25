//! CSG evaluation over the feature timeline (`crate::timeline::Timeline`)
//! — solids built through manifold3d + stable face naming per M6.
//!
//! The model itself is typed feature data with stable ids (timeline.rs);
//! this module lowers it to geometry: each feature is evaluated exactly
//! once (DAG semantics: operand FeatureIds resolve to earlier positions
//! via the timeline index), and every position publishes a
//! `naming::FaceTable` — the analytic surfaces its named faces live on.
//! CSG nodes *merge* operand tables (first operand wins coincident
//! ties); a cut wall therefore inherits the TOOL feature's name, and
//! `move` translates the operand's surfaces. The root solid's kernel
//! triangles are then re-identified against the root table
//! (`naming::name_faces`), which is what keeps named faces bound across
//! parameter edits and mid-timeline insertions — see docs/NAMING.md.
//!
//! The tree is typed Rust data (mission rule): operands are
//! `FeatureId`s, so evaluation order, sharing and identity are
//! compiler/test-visible, and a model file parses into a timeline, not
//! a string soup.
//!
//! Precision contract: primitives are tessellated in f64 (`tess`), fed
//! to manifold via `from_mesh_f64`, and volume/genus/face naming run in
//! f64. Only the *display* mesh narrows to f32 (`Mesh`), and STL is f32
//! by format.

use std::collections::HashMap;

use glam::DVec3;
use manifold3d::Manifold;

use crate::naming::{self, FaceRecord, FaceTable, NamedFace};
use crate::primitives::Mesh;
use crate::sketch::Sketch;
use crate::sketch::loops::{Sample, regions};
use crate::sketch::solver::{SolveStatus, solve};
use crate::sweep::{Axis2, extrude_region, revolve_region};
use crate::tess::{Mesh64, box64, cylinder64, sphere64, torus64};
// The data model lives in `timeline` (M6); re-exported here so the
// evaluator and its input model stay one import for consumers.
pub use crate::timeline::{FeatureId, GeoNode, Timeline};

/// Kernel failure, flattened to a message suitable for CLI/report output.
#[derive(Clone, Debug)]
pub struct KernelError(pub String);

impl std::fmt::Display for KernelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for KernelError {}

/// Evaluated solid: display mesh (f32), exact-kernel volume (f64, mm^3),
/// topological genus, triangle count, and the M6 named-face table.
#[derive(Clone, Debug)]
pub struct Solid {
    pub mesh: Mesh,
    pub volume_mm3: f64,
    pub genus: i32,
    pub num_tri: usize,
    /// M6: named faces of the root solid, sorted by
    /// (feature id, tag, component). Unmatched kernel triangles group
    /// under key `F0:unmatched` — naming failures stay visible.
    pub faces: Vec<NamedFace>,
}

impl Solid {
    /// Mass in grams for a material density in g/cm^3 (model units are mm:
    /// 1 cm^3 = 1000 mm^3).
    pub fn mass_g(&self, density_g_cm3: f64) -> f64 {
        self.volume_mm3 * density_g_cm3 / 1000.0
    }

    /// M6: the face whose key matches exactly (component suffixes
    /// included), if any. The re-bind tests assert on this.
    pub fn face(&self, key: &str) -> Option<&NamedFace> {
        self.faces.iter().find(|f| f.key == key)
    }
}

/// Welded -> deduplicated f64 vertex arrays for the kernel: exact bit keys
/// (M2 lesson: only feed the kernel positions that already match bitwise).
fn to_kernel_mesh(m: &Mesh64) -> (Vec<f64>, Vec<u64>) {
    let mut map: HashMap<[u64; 3], u32> = HashMap::new();
    let mut verts: Vec<f64> = Vec::with_capacity(m.positions.len() * 3);
    let mut remap = Vec::with_capacity(m.positions.len());
    for p in &m.positions {
        let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
        let id = if let Some(&id) = map.get(&key) {
            id
        } else {
            let id = map.len() as u32;
            map.insert(key, id);
            verts.extend_from_slice(p);
            id
        };
        remap.push(id as u64);
    }
    let tris: Vec<u64> = m.indices.iter().map(|i| remap[*i as usize]).collect();
    (verts, tris)
}

fn mesh_to_manifold(m: &Mesh64) -> Result<Manifold, KernelError> {
    let (verts, tris) = to_kernel_mesh(m);
    Manifold::from_mesh_f64(&verts, 3, &tris)
        .map_err(|e| KernelError(format!("kernel rejected primitive mesh: {e}")))
}

/// Kernel output in f64: welded positions + triangle index triples
/// (welded by construction — manifold deduplicates vertices itself).
fn kernel_mesh_f64(manifold: &Manifold) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    let (props, n_props, tris) = manifold.to_mesh_f64();
    debug_assert!(n_props >= 3);
    let nv = tris.iter().map(|&i| i as usize).max().map_or(0, |m| m + 1);
    let mut verts: Vec<DVec3> = Vec::with_capacity(nv);
    for k in 0..nv {
        let b = k * n_props;
        verts.push(DVec3::new(props[b], props[b + 1], props[b + 2]));
    }
    let out = tris
        .chunks(3)
        .map(|t| [t[0] as u32, t[1] as u32, t[2] as u32])
        .collect();
    (verts, out)
}

/// Flat-shaded triangle soup from welded f64 kernel output: one vertex
/// per face corner with the face normal (kernel output welds vertices;
/// display wants hard edges, and STL carries face normals anyway).
fn display_from_kernel(verts: &[DVec3], tris: &[[u32; 3]]) -> Mesh {
    let mut mesh = Mesh::new();
    for t in tris {
        let p = [
            verts[t[0] as usize],
            verts[t[1] as usize],
            verts[t[2] as usize],
        ];
        let n = (p[1] - p[0]).cross(p[2] - p[0]);
        let n = if n.length_squared() > 0.0 {
            n.normalize()
        } else {
            DVec3::Y
        };
        for v in p {
            let idx = mesh.positions.len() as u32;
            mesh.positions.push(v.as_vec3().to_array());
            mesh.normals.push(n.as_vec3().to_array());
            mesh.indices.push(idx);
        }
    }
    mesh
}

/// Cached-manifold lookup with a kernel-level error (internal invariant:
/// operands are always evaluated before their parent node).
fn get(cache: &[Option<Manifold>], i: usize) -> Result<&Manifold, KernelError> {
    cache[i]
        .as_ref()
        .ok_or_else(|| KernelError(format!("operand #{i} not evaluated")))
}

/// Validate primitive dimensions before tessellation: file input is
/// untrusted, and the tessellators assert (debug) — the library path must
/// answer with an error instead.
fn dim_ok(name: &str, pairs: &[(&str, f64)]) -> Result<(), KernelError> {
    for (what, v) in pairs {
        if !v.is_finite() || *v <= 0.0 {
            return Err(KernelError(format!(
                "{name}: {what} must be > 0 and finite (got {v})"
            )));
        }
    }
    Ok(())
}

/// Solve a sketch and extract closed regions. `profile_segs` 0 = chord
/// tolerance sampling (renderer behaviour); N = N segments per full turn
/// (what the exact-polyhedral gates rely on). Also returns the SOLVED
/// sketch so naming can declare wall surfaces from entity parameters
/// without re-solving (identical result — the solver is deterministic).
fn sketch_regions(
    sk: &Sketch,
    profile_segs: usize,
    skip_entities: &[usize],
) -> Result<(Vec<crate::sketch::loops::Region2>, Sketch), KernelError> {
    let (solved, rep) = solve(sk);
    if rep.status != SolveStatus::Solved {
        return Err(KernelError(format!(
            "sketch {}: solver did not converge (max|res| {:.3e}): {}",
            sk.name, rep.max_residual, rep.message
        )));
    }
    let sample = if profile_segs > 0 {
        Sample::Segs(profile_segs)
    } else {
        Sample::Chord(0.02)
    };
    let regs = regions(&solved, sample, 1e-9, skip_entities)
        .map_err(|e| KernelError(format!("sketch {}: {e}", sk.name)))?;
    Ok((regs, solved))
}

fn find_sketch<'a>(
    sketches: &'a [Sketch],
    name: &str,
    node: &str,
) -> Result<&'a Sketch, KernelError> {
    sketches.iter().find(|s| s.name == name).ok_or_else(|| {
        KernelError(format!(
            "node {node}: sketch {name:?} not defined in this model"
        ))
    })
}

/// Coordinates of a point entity (kind was checked at add time).
fn point_xy(sk: &Sketch, i: usize) -> Result<[f64; 2], KernelError> {
    match sk.entities.get(i).map(|(_, e)| e) {
        Some(crate::sketch::Entity::Point { x, y }) => Ok([*x, *y]),
        _ => Err(KernelError(format!(
            "sketch {}: entity #{i} is not a point",
            sk.name
        ))),
    }
}

/// Sweep a region set through our own mesh builders, insist our
/// closed-manifold check passes BEFORE the kernel sees it, and union the
/// per-region solids into one manifold (sketch may hold several regions).
fn swept_manifold(
    name: &str,
    regs: &[crate::sketch::loops::Region2],
    build: impl Fn(&crate::sketch::loops::Region2) -> Result<Mesh64, crate::sweep::SweepError>,
) -> Result<Manifold, KernelError> {
    let mut acc: Option<Manifold> = None;
    for (ri, r) in regs.iter().enumerate() {
        let m = build(r).map_err(|e| KernelError(format!("{name}: region #{ri}: {e}")))?;
        if let Err(e) = m.closed_manifold_check() {
            return Err(KernelError(format!(
                "{name}: generated sweep mesh failed closed-manifold check: {e}"
            )));
        }
        let man = mesh_to_manifold(&m)?;
        acc = Some(match acc {
            None => man,
            Some(prev) => prev.union(&man),
        });
    }
    acc.ok_or_else(|| KernelError(format!("{name}: sketch produced no regions")))
}

/// Merge two operand tables (M6 convention: operand A first — coincident
/// surfaces tie-break deterministically toward A; docs/NAMING.md FM-3).
fn merged_table(a: &FaceTable, b: &FaceTable) -> FaceTable {
    let mut t = a.clone();
    t.extend(b.iter().cloned());
    t
}

/// Evaluate the whole timeline; each feature is built exactly once and
/// shared features reuse their kernel manifold (DAG semantics). Sketch-
/// backed features resolve their sketch by name from `sketches`.
pub fn evaluate_with(graph: &Timeline, sketches: &[Sketch]) -> Result<Solid, KernelError> {
    let root = graph
        .root_pos()
        .ok_or_else(|| KernelError("empty timeline".into()))?;
    let mut cache: Vec<Option<Manifold>> = (0..graph.features.len()).map(|_| None).collect();
    let mut tables: Vec<FaceTable> = vec![Vec::new(); graph.features.len()];
    let pos_of = |fid: FeatureId| -> Result<usize, KernelError> {
        graph
            .pos_of(fid)
            .ok_or_else(|| KernelError(format!("operand {fid} is not defined in this timeline")))
    };
    for i in 0..=root {
        let feat = &graph.features[i];
        let name = feat.name.clone();
        let id = feat.id;
        let built = match &feat.op {
            GeoNode::Box { w, h, d } => {
                dim_ok(&name, &[("width", *w), ("height", *h), ("depth", *d)])?;
                tables[i] = naming::records_box(id, *w, *h, *d);
                mesh_to_manifold(&box64(*w, *h, *d))?
            }
            GeoNode::Cylinder { r, h, segs } => {
                dim_ok(&name, &[("radius", *r), ("height", *h)])?;
                if *segs < 3 {
                    return Err(KernelError(format!(
                        "{name}: cylinder segments must be >= 3"
                    )));
                }
                tables[i] = naming::records_cylinder(id, *r, *h);
                mesh_to_manifold(&cylinder64(*r, *h, *segs))?
            }
            GeoNode::Sphere { r, rings, segs } => {
                dim_ok(&name, &[("radius", *r)])?;
                if *rings < 2 || *segs < 3 {
                    return Err(KernelError(format!(
                        "{name}: sphere needs rings>=2, segs>=3"
                    )));
                }
                tables[i] = naming::records_sphere(id, *r);
                mesh_to_manifold(&sphere64(*r, *rings, *segs))?
            }
            GeoNode::Torus {
                r,
                tube,
                major_segs,
                minor_segs,
            } => {
                dim_ok(&name, &[("ring radius", *r), ("tube radius", *tube)])?;
                if *major_segs < 3 || *minor_segs < 3 {
                    return Err(KernelError(format!(
                        "{name}: torus needs major/minor segs >= 3"
                    )));
                }
                tables[i] = naming::records_torus(id, *r, *tube);
                mesh_to_manifold(&torus64(*r, *tube, *major_segs, *minor_segs))?
            }
            GeoNode::Extrude {
                sketch,
                height,
                offset,
                profile_segs,
            } => {
                dim_ok(&name, &[("extrude height", *height)])?;
                if !offset.is_finite() {
                    return Err(KernelError(format!("{name}: offset must be finite")));
                }
                let sk = find_sketch(sketches, sketch, &name)?;
                let (regs, solved) = sketch_regions(sk, *profile_segs, &[])?;
                let (z0, z1) = (*offset, *offset + *height);
                tables[i] = naming::records_extrude(id, &solved, z0, z1);
                swept_manifold(&name, &regs, |r| extrude_region(r, z0, z1))?
            }
            GeoNode::Revolve {
                sketch,
                axis,
                angle,
                sweep_segs,
                profile_segs,
            } => {
                if *sweep_segs < 3 {
                    return Err(KernelError(format!(
                        "{name}: revolve sweep segments must be >= 3"
                    )));
                }
                let sk = find_sketch(sketches, sketch, &name)?;
                // the axis line is a construction entity: excluded from
                // region extraction (same skip set M5 used)
                let (solved_pre, rep) = solve(sk);
                if rep.status != SolveStatus::Solved {
                    return Err(KernelError(format!(
                        "sketch {}: solver did not converge (max|res| {:.3e}): {}",
                        sk.name, rep.max_residual, rep.message
                    )));
                }
                let ai = solved_pre.entity_index(axis).ok_or_else(|| {
                    KernelError(format!(
                        "{name}: axis entity {axis:?} not in sketch {sketch}"
                    ))
                })?;
                let (pa, pb) = match solved_pre.entities[ai].1 {
                    crate::sketch::Entity::Line { a, b } => {
                        (point_xy(&solved_pre, a)?, point_xy(&solved_pre, b)?)
                    }
                    _ => {
                        return Err(KernelError(format!(
                            "{name}: revolve axis {axis:?} must be a line entity"
                        )));
                    }
                };
                let axis2 =
                    Axis2::through(pa, pb).map_err(|e| KernelError(format!("{name}: {e}")))?;
                let (regs, solved) = sketch_regions(sk, *profile_segs, &[ai])?;
                tables[i] = naming::records_revolve(id, &solved, axis2, *angle, axis, &regs);
                swept_manifold(&name, &regs, |r| {
                    revolve_region(r, axis2, *angle, *sweep_segs)
                })?
            }
            GeoNode::Move { node, x, y, z } => {
                let p = pos_of(*node)?;
                let t = DVec3::new(*x, *y, *z);
                tables[i] = tables[p]
                    .iter()
                    .map(|r| FaceRecord {
                        surface: r.surface.translate(t),
                        ..r.clone()
                    })
                    .collect();
                cache[p]
                    .as_ref()
                    .ok_or_else(|| KernelError(format!("operand #{p} not evaluated")))?
                    .translate(*x, *y, *z)
            }
            GeoNode::Union { a, b } => {
                let (pa, pb) = (pos_of(*a)?, pos_of(*b)?);
                tables[i] = merged_table(&tables[pa], &tables[pb]);
                let ca = get(&cache, pa)?;
                let cb = get(&cache, pb)?;
                ca.union(cb)
            }
            GeoNode::Cut { a, b } => {
                let (pa, pb) = (pos_of(*a)?, pos_of(*b)?);
                tables[i] = merged_table(&tables[pa], &tables[pb]);
                let ca = get(&cache, pa)?;
                let cb = get(&cache, pb)?;
                ca.difference(cb)
            }
            GeoNode::Intersect { a, b } => {
                let (pa, pb) = (pos_of(*a)?, pos_of(*b)?);
                tables[i] = merged_table(&tables[pa], &tables[pb]);
                let ca = get(&cache, pa)?;
                let cb = get(&cache, pb)?;
                ca.intersection(cb)
            }
        };
        cache[i] = Some(built);
    }
    let root_m = cache[root]
        .as_ref()
        .ok_or_else(|| KernelError("root not evaluated".into()))?;
    let volume_mm3 = root_m.volume();
    let genus = root_m.genus();
    let num_tri = root_m.num_tri();
    let (verts, tris) = kernel_mesh_f64(root_m);
    // M6: re-identify the rebuilt mesh against the declaring face table.
    let faces = naming::name_faces(&verts, &tris, &tables[root]);
    let mesh = display_from_kernel(&verts, &tris);
    if let Err(why) = mesh.closed_manifold_check() {
        // Kernel guarantees manifoldness; if this ever fires we have a
        // conversion bug or corrupted data, not a geometry bug — fail
        // loudly, never ship it. Carry BOTH counts and the sub-check
        // text: equal counts + unpaired edges => the mesh bytes changed
        // between kernel and audit (process memory corruption — this
        // box has documented bad-page-state events); display != kernel
        // tris => conversion bug in display_from_kernel; a specific
        // (a,b) edge => geometry bug. (2026-11-24: the first version
        // dropped `why` and one red run was un-forensicable.)
        return Err(KernelError(format!(
            "internal: kernel output failed closed-manifold check \
             (kernel {num_tri} tris, display {} tris): {why}",
            mesh.triangle_count()
        )));
    }
    Ok(Solid {
        mesh,
        volume_mm3,
        genus,
        num_tri,
        faces,
    })
}

/// Compatibility wrapper for pure-geometry timelines (no sketch nodes).
pub fn evaluate(graph: &Timeline) -> Result<Solid, KernelError> {
    evaluate_with(graph, &[])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g() -> Timeline {
        Timeline::default()
    }

    #[test]
    fn push_rejects_bad_names_and_forward_refs() {
        let mut gr = g();
        gr.push(
            "a",
            GeoNode::Box {
                w: 1.0,
                h: 1.0,
                d: 1.0,
            },
        )
        .unwrap();
        assert!(
            gr.push(
                "a",
                GeoNode::Box {
                    w: 2.0,
                    h: 1.0,
                    d: 1.0
                }
            )
            .is_err()
        );
        // operand id 5 was never defined -> rejected
        assert!(
            gr.push(
                "b",
                GeoNode::Union {
                    a: FeatureId(5),
                    b: FeatureId(1)
                }
            )
            .is_err()
        );
    }

    #[test]
    fn union_of_disjoint_boxes_sums_volumes() {
        let mut gr = g();
        let a = gr
            .push(
                "a",
                GeoNode::Box {
                    w: 2.0,
                    h: 2.0,
                    d: 2.0,
                },
            )
            .unwrap();
        let m = gr
            .push(
                "m",
                GeoNode::Move {
                    node: a,
                    x: 3.0,
                    y: 0.0,
                    z: 0.0,
                },
            )
            .unwrap();
        gr.push("u", GeoNode::Union { a, b: m }).unwrap();
        let s = evaluate(&gr).unwrap();
        assert!((s.volume_mm3 - 16.0).abs() < 1e-9);
        // manifold's genus for a DISCONNECTED 2-component union is -1
        // (single-shell formula 1 - chi/2 with chi = 4); verified, not a bug.
        assert_eq!(s.genus, -1);
    }

    #[test]
    fn cut_hole_gives_exact_box_minus_prism_volume_and_genus() {
        // plate 30x10x20 with through-bore r=5, h=14 (protrudes both sides)
        let mut gr = g();
        let plate = gr
            .push(
                "plate",
                GeoNode::Box {
                    w: 30.0,
                    h: 10.0,
                    d: 20.0,
                },
            )
            .unwrap();
        let bore = gr
            .push(
                "bore",
                GeoNode::Cylinder {
                    r: 5.0,
                    h: 14.0,
                    segs: 32,
                },
            )
            .unwrap();
        gr.push("part", GeoNode::Cut { a: plate, b: bore }).unwrap();
        let s = evaluate(&gr).unwrap();
        let inscribed = 0.5 * 32.0 * 25.0 * (TAU / 32.0).sin() * 10.0;
        assert!((s.volume_mm3 - (6000.0 - inscribed)).abs() < 1e-9);
        assert_eq!(s.genus, 1, "through-hole must make genus 1");
    }

    const TAU: f64 = std::f64::consts::TAU;

    #[test]
    fn intersect_keeps_common_region() {
        let mut gr = g();
        let a = gr
            .push(
                "a",
                GeoNode::Box {
                    w: 4.0,
                    h: 4.0,
                    d: 4.0,
                },
            )
            .unwrap();
        let b = gr
            .push(
                "b",
                GeoNode::Box {
                    w: 2.0,
                    h: 8.0,
                    d: 2.0,
                },
            )
            .unwrap();
        gr.push("i", GeoNode::Intersect { a, b }).unwrap();
        let s = evaluate(&gr).unwrap();
        assert!(
            (s.volume_mm3 - 16.0).abs() < 1e-9,
            "intersection is 2 x 4 x 2"
        );
    }

    #[test]
    fn dag_sharing_does_not_double_evaluate() {
        // u = union(a, m) where m = move of a: a is referenced twice.
        let mut gr = g();
        let a = gr
            .push(
                "a",
                GeoNode::Box {
                    w: 1.0,
                    h: 1.0,
                    d: 1.0,
                },
            )
            .unwrap();
        let m = gr
            .push(
                "m",
                GeoNode::Move {
                    node: a,
                    x: 2.0,
                    y: 0.0,
                    z: 0.0,
                },
            )
            .unwrap();
        gr.push("u", GeoNode::Union { a, b: m }).unwrap();
        let s = evaluate(&gr).unwrap();
        assert!((s.volume_mm3 - 2.0).abs() < 1e-9);
    }

    /// The M6 face table on a bare primitive: all six box faces must be
    /// named and match their declaring planes (this is the unit that the
    /// cross-rebuild identity tests build on).
    #[test]
    fn box_root_gets_six_named_faces() {
        let mut gr = g();
        gr.push(
            "block",
            GeoNode::Box {
                w: 10.0,
                h: 4.0,
                d: 2.0,
            },
        )
        .unwrap();
        let s = evaluate(&gr).unwrap();
        assert_eq!(s.faces.len(), 6, "box faces: {:?}", keys(&s));
        assert!(s.face("F1:face:+x").is_some());
        assert!(s.face("F1:face:-x").is_some());
        assert!(s.face("F1:face:+y").is_some());
        assert!(s.face("F1:face:-y").is_some());
        assert!(s.face("F1:face:+z").is_some());
        assert!(s.face("F1:face:-z").is_some());
        assert!(!keys(&s).iter().any(|k| k.contains("unmatched")));
    }

    fn keys(s: &Solid) -> Vec<String> {
        s.faces.iter().map(|f| f.key.clone()).collect()
    }

    /// The M3 gate torus (R=0.3, tube=0.06 @1024x1024) through the full
    /// kernel path must pass the closed-manifold audit inside `evaluate`,
    /// and its counts must be exactly the requested tessellation. Locked
    /// 2026-11-24 after one non-reproducible red run (docs/MILESTONES.md,
    /// M3 repair): the audit is deterministic and green 18/18 probe
    /// rounds, so any future fire is real corruption or a real geometry
    /// bug, and the message now says which. M6: every triangle must also
    /// re-identify onto the torus surface record (1 face, 0 unmatched).
    #[test]
    fn torus_gate_resolution_stays_closed_manifold() {
        let mut gr = g();
        gr.push(
            "ring",
            GeoNode::Torus {
                r: 0.3,
                tube: 0.06,
                major_segs: 1024,
                minor_segs: 1024,
            },
        )
        .unwrap();
        let s = evaluate(&gr).expect("gate-resolution torus must stay closed-manifold");
        assert_eq!(s.num_tri, 2 * 1024 * 1024, "kernel tri count == 2*segs^2");
        assert_eq!(
            s.mesh.triangle_count(),
            s.num_tri,
            "display tris must match kernel tris"
        );
        assert_eq!(s.genus, 1);
        assert_eq!(s.faces.len(), 1, "one torus surface: {:?}", keys(&s));
        assert_eq!(s.faces[0].num_tris, s.num_tri);
    }
}
