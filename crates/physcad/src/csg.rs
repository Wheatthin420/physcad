//! CSG tree (union / cut / intersect) over solids, backed by manifold3d.
//!
//! The tree is typed Rust data (mission rule): nodes reference earlier
//! nodes by index, so evaluation order and sharing are compiler/test-
//! visible and a model file parses into a DAG, not a string soup.
//!
//! Precision contract: primitives are tessellated in f64 (`tess`), fed to
//! manifold via `from_mesh_f64`, and volume/genus are read in f64. Only
//! the *display* mesh narrows to f32 (`Mesh`), and STL is f32 by format.

use std::collections::{BTreeMap, HashMap};

use manifold3d::Manifold;

use crate::primitives::Mesh;
use crate::tess::{Mesh64, box64, cylinder64, sphere64};

/// One node of the geometry graph. Operands are indices into the same
/// `GeoGraph::nodes` vector and are always < the node's own index (the
/// graph is in dependency order by construction in `model`).
#[derive(Clone, Debug, PartialEq)]
pub enum GeoNode {
    Box {
        w: f64,
        h: f64,
        d: f64,
    },
    Cylinder {
        r: f64,
        h: f64,
        segs: usize,
    },
    Sphere {
        r: f64,
        rings: usize,
        segs: usize,
    },
    Move {
        node: usize,
        x: f64,
        y: f64,
        z: f64,
    },
    Union {
        a: usize,
        b: usize,
    },
    /// `a` minus `b`.
    Cut {
        a: usize,
        b: usize,
    },
    Intersect {
        a: usize,
        b: usize,
    },
}

/// Ordered geometry graph: names, nodes, and the root (last node).
#[derive(Clone, Debug, Default)]
pub struct GeoGraph {
    pub nodes: Vec<(String, GeoNode)>,
    by_name: BTreeMap<String, usize>,
}

impl GeoGraph {
    pub fn push(&mut self, name: &str, node: GeoNode) -> Result<usize, String> {
        if let Some(prev) = self.by_name.get(name) {
            return Err(format!(
                "node {name:?} defined twice (nodes {} and {})",
                prev,
                self.nodes.len()
            ));
        }
        let idx = self.nodes.len();
        // Dependency-order invariant: operands must reference earlier nodes.
        for op in Self::operands(&node) {
            if op >= idx {
                return Err(format!(
                    "node {name} references operand #{op} before definition"
                ));
            }
        }
        self.nodes.push((name.to_string(), node));
        self.by_name.insert(name.to_string(), idx);
        Ok(idx)
    }

    pub fn find(&self, name: &str) -> Option<usize> {
        self.by_name.get(name).copied()
    }

    pub fn root(&self) -> Option<usize> {
        if self.nodes.is_empty() {
            None
        } else {
            Some(self.nodes.len() - 1)
        }
    }

    fn operands(node: &GeoNode) -> Vec<usize> {
        match node {
            GeoNode::Move { node, .. } => vec![*node],
            GeoNode::Union { a, b } | GeoNode::Cut { a, b } | GeoNode::Intersect { a, b } => {
                vec![*a, *b]
            }
            _ => vec![],
        }
    }
}

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
/// topological genus, and triangle count.
#[derive(Clone, Debug)]
pub struct Solid {
    pub mesh: Mesh,
    pub volume_mm3: f64,
    pub genus: i32,
    pub num_tri: usize,
}

impl Solid {
    /// Mass in grams for a material density in g/cm^3 (model units are mm:
    /// 1 cm^3 = 1000 mm^3).
    pub fn mass_g(&self, density_g_cm3: f64) -> f64 {
        self.volume_mm3 * density_g_cm3 / 1000.0
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

/// Flat-shaded triangle soup out of the kernel's f64 mesh: one vertex per
/// face corner with the face normal (kernel output welds vertices; display
/// wants hard edges, and STL carries face normals anyway).
fn kernel_to_display(manifold: &Manifold) -> Mesh {
    let (props, n_props, tris) = manifold.to_mesh_f64();
    debug_assert!(n_props >= 3);
    let mut mesh = Mesh::new();
    for tri in tris.chunks(3) {
        let p: [glam::DVec3; 3] = std::array::from_fn(|k| {
            let b = tri[k] as usize * n_props;
            glam::DVec3::new(props[b], props[b + 1], props[b + 2])
        });
        let n = (p[1] - p[0]).cross(p[2] - p[0]);
        let n = if n.length_squared() > 0.0 {
            n.normalize()
        } else {
            glam::DVec3::Y
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

/// Evaluate the whole graph; each node is built exactly once and shared
/// nodes reuse their kernel manifold (DAG semantics).
pub fn evaluate(graph: &GeoGraph) -> Result<Solid, KernelError> {
    let root = graph
        .root()
        .ok_or_else(|| KernelError("empty geometry graph".into()))?;
    let mut cache: Vec<Option<Manifold>> = (0..graph.nodes.len()).map(|_| None).collect();
    for i in 0..=root {
        let name = &graph.nodes[i].0;
        let built = match &graph.nodes[i].1 {
            GeoNode::Box { w, h, d } => {
                dim_ok(name, &[("width", *w), ("height", *h), ("depth", *d)])?;
                mesh_to_manifold(&box64(*w, *h, *d))?
            }
            GeoNode::Cylinder { r, h, segs } => {
                dim_ok(name, &[("radius", *r), ("height", *h)])?;
                if *segs < 3 {
                    return Err(KernelError(format!(
                        "{name}: cylinder segments must be >= 3"
                    )));
                }
                mesh_to_manifold(&cylinder64(*r, *h, *segs))?
            }
            GeoNode::Sphere { r, rings, segs } => {
                dim_ok(name, &[("radius", *r)])?;
                if *rings < 2 || *segs < 3 {
                    return Err(KernelError(format!(
                        "{name}: sphere needs rings>=2, segs>=3"
                    )));
                }
                mesh_to_manifold(&sphere64(*r, *rings, *segs))?
            }
            GeoNode::Move { node, x, y, z } => cache[*node]
                .as_ref()
                .ok_or_else(|| KernelError(format!("operand #{node} not evaluated")))?
                .translate(*x, *y, *z),
            GeoNode::Union { a, b } => {
                let ca = get(&cache, *a)?;
                let cb = get(&cache, *b)?;
                ca.union(cb)
            }
            GeoNode::Cut { a, b } => {
                let ca = get(&cache, *a)?;
                let cb = get(&cache, *b)?;
                ca.difference(cb)
            }
            GeoNode::Intersect { a, b } => {
                let ca = get(&cache, *a)?;
                let cb = get(&cache, *b)?;
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
    let mesh = kernel_to_display(root_m);
    if mesh.closed_manifold_check().is_err() {
        // Kernel guarantees manifoldness; if this ever fires we have a
        // conversion bug, not a geometry bug — fail loudly, never ship it.
        return Err(KernelError(format!(
            "internal: kernel output failed closed-manifold check ({} tris)",
            mesh.triangle_count()
        )));
    }
    Ok(Solid {
        mesh,
        volume_mm3,
        genus,
        num_tri,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g() -> GeoGraph {
        GeoGraph::default()
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
        // operand index 5 >= node index 1 -> forward reference rejected
        assert!(gr.push("b", GeoNode::Union { a: 5, b: 0 }).is_err());
    }

    #[test]
    fn union_of_disjoint_boxes_sums_volumes() {
        let mut gr = g();
        gr.push(
            "a",
            GeoNode::Box {
                w: 2.0,
                h: 2.0,
                d: 2.0,
            },
        )
        .unwrap();
        gr.push(
            "b",
            GeoNode::Move {
                node: 0,
                x: 3.0,
                y: 0.0,
                z: 0.0,
            },
        )
        .unwrap();
        gr.push("u", GeoNode::Union { a: 0, b: 1 }).unwrap();
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
        gr.push(
            "plate",
            GeoNode::Box {
                w: 30.0,
                h: 10.0,
                d: 20.0,
            },
        )
        .unwrap();
        gr.push(
            "bore",
            GeoNode::Cylinder {
                r: 5.0,
                h: 14.0,
                segs: 32,
            },
        )
        .unwrap();
        gr.push("part", GeoNode::Cut { a: 0, b: 1 }).unwrap();
        let s = evaluate(&gr).unwrap();
        let inscribed = 0.5 * 32.0 * 25.0 * (TAU / 32.0).sin() * 10.0;
        assert!((s.volume_mm3 - (6000.0 - inscribed)).abs() < 1e-9);
        assert_eq!(s.genus, 1, "through-hole must make genus 1");
    }

    const TAU: f64 = std::f64::consts::TAU;

    #[test]
    fn intersect_keeps_common_region() {
        let mut gr = g();
        gr.push(
            "a",
            GeoNode::Box {
                w: 4.0,
                h: 4.0,
                d: 4.0,
            },
        )
        .unwrap();
        gr.push(
            "b",
            GeoNode::Box {
                w: 2.0,
                h: 8.0,
                d: 2.0,
            },
        )
        .unwrap();
        gr.push("i", GeoNode::Intersect { a: 0, b: 1 }).unwrap();
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
        gr.push(
            "a",
            GeoNode::Box {
                w: 1.0,
                h: 1.0,
                d: 1.0,
            },
        )
        .unwrap();
        gr.push(
            "m",
            GeoNode::Move {
                node: 0,
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
        )
        .unwrap();
        gr.push("u", GeoNode::Union { a: 0, b: 1 }).unwrap();
        let s = evaluate(&gr).unwrap();
        assert!((s.volume_mm3 - 2.0).abs() < 1e-9);
    }
}
