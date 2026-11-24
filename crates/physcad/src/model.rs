//! `.pcm` model files: text -> parameters + CSG graph.
//!
//! Grammar (one statement per line; `#` starts a comment):
//!
//! ```text
//!   param NAME = EXPR              # numeric parameter, EXPR may use earlier params
//!   density = EXPR                 # material readout, g/cm^3 (geometry-irrelevant)
//!   NAME = box(W, H, D)
//!   NAME = cylinder(R, H, SEGS)
//!   NAME = sphere(R, RINGS, SEGS)
//!   NAME = move(NODE, X, Y, Z)
//!   NAME = union(A, B) | cut(A, B) | intersect(A, B)
//! ```
//!
//! The root of the solid is the LAST node defined. Units: millimetres.
//! Node operands are plain names and must already be defined (files read
//! top-to-bottom — declaration order is the dependency order, typed as a
//! GeoGraph DAG instead of parsed at draw time).

use std::collections::BTreeMap;
use std::fmt;

use crate::csg::{GeoGraph, GeoNode};
use crate::expr;

#[derive(Clone, Debug)]
pub struct ModelError {
    pub file: String,
    pub line: usize,
    pub col: usize,
    pub msg: String,
}

impl fmt::Display for ModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}:{}: {}", self.file, self.line, self.col, self.msg)
    }
}

impl std::error::Error for ModelError {}

#[derive(Clone, Debug)]
pub struct Model {
    pub name: String,
    pub density_g_cm3: f64,
    pub graph: GeoGraph,
}

/// Split arg list at top-level commas (nesting-aware; exprs may contain
/// their own parentheses).
fn split_args(s: &str) -> Result<Vec<String>, (usize, String)> {
    let mut depth = 0i32;
    let mut cur = String::new();
    let mut out = Vec::new();
    for ch in s.chars() {
        match ch {
            '(' => {
                depth += 1;
                cur.push(ch);
            }
            ')' => {
                depth -= 1;
                if depth < 0 {
                    return Err((0, "unbalanced ')' in call".into()));
                }
                cur.push(ch);
            }
            ',' if depth == 0 => {
                out.push(std::mem::take(&mut cur));
            }
            _ => cur.push(ch),
        }
    }
    if depth != 0 {
        return Err((0, "unbalanced '(' in call".into()));
    }
    out.push(cur);
    Ok(out)
}

/// `NAME = call(...)` right-hand side: returns (call_name, arg_strings).
fn parse_call(rhs: &str) -> Result<(String, Vec<String>), (usize, String)> {
    let t = rhs.trim();
    let open = t
        .find('(')
        .ok_or((1usize, "expected NAME = call(args...)".to_string()))?;
    let head = t[..open].trim();
    if !t.ends_with(')') {
        return Err((t.len(), "call must end with ')'".into()));
    }
    let inner = &t[open + 1..t.len() - 1];
    let args = split_args(inner)?;
    Ok((head.to_string(), args))
}

fn ident(s: &str) -> Option<String> {
    let s = s.trim();
    let mut c = s.chars();
    let first = c.next()?;
    if !(first.is_ascii_alphabetic() || first == '_') {
        return None;
    }
    if c.all(|ch| ch.is_ascii_alphanumeric() || ch == '_') {
        Some(s.to_string())
    } else {
        None
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ArgKind {
    Num,
    Node,
}

/// Parse model source into parameters + a GeoGraph (no kernel work yet).
pub fn parse_model(src: &str, file: &str) -> Result<Model, ModelError> {
    let mut graph = GeoGraph::default();
    let mut params: BTreeMap<String, f64> = BTreeMap::new();
    let mut density: Option<f64> = None;
    let mut node_count = 0usize;

    let err = |line: usize, col: usize, msg: String| ModelError {
        file: file.to_string(),
        line,
        col,
        msg,
    };

    for (lineno, raw) in src.lines().enumerate().map(|(i, l)| (i + 1, l)) {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let Some(eq) = line.find('=') else {
            return Err(err(
                lineno,
                1,
                format!("expected 'name = ...', got {line:?}"),
            ));
        };
        let lhs = line[..eq].trim();
        let rhs = &line[eq + 1..];

        if lhs == "density" {
            let e = expr::parse(rhs).map_err(|e| err(lineno, e.col, e.msg))?;
            let v = e.eval(&params).map_err(|e| err(lineno, e.col, e.msg))?;
            if v <= 0.0 || !v.is_finite() {
                return Err(err(
                    lineno,
                    1,
                    "density must be a positive finite number".to_string(),
                ));
            }
            density = Some(v);
            continue;
        }

        if let Some(rest) = lhs.strip_prefix("param") {
            let Some(name) = ident(rest) else {
                return Err(err(
                    lineno,
                    1,
                    format!("bad parameter name {:?}", rest.trim()),
                ));
            };
            let e = expr::parse(rhs).map_err(|e| err(lineno, e.col, e.msg))?;
            let v = e.eval(&params).map_err(|e| err(lineno, e.col, e.msg))?;
            if !v.is_finite() {
                return Err(err(lineno, 1, format!("param {name} is not finite")));
            }
            params.insert(name, v);
            continue;
        }

        let Some(name) = ident(lhs) else {
            return Err(err(lineno, 1, format!("bad node name {lhs:?}")));
        };
        let (call, args) = parse_call(rhs).map_err(|(col, m)| err(lineno, eq + 1 + col, m))?;

        // Per-call argument signature: numeric expressions vs node names.
        // Evaluating a node-name slot as an expression would surface the
        // wrong error ("unknown variable"), so classify before evaluating.
        let sig: &[ArgKind] = match call.as_str() {
            "box" | "cylinder" | "sphere" => &[ArgKind::Num, ArgKind::Num, ArgKind::Num],
            "move" => &[ArgKind::Node, ArgKind::Num, ArgKind::Num, ArgKind::Num],
            "union" | "cut" | "intersect" => &[ArgKind::Node, ArgKind::Node],
            other => {
                return Err(err(
                    lineno,
                    1,
                    format!("unknown geometry call {other:?} (args: {})", args.len()),
                ));
            }
        };
        if args.len() != sig.len() {
            return Err(err(
                lineno,
                1,
                format!(
                    "{}() takes {} arguments, got {}",
                    call,
                    sig.len(),
                    args.len()
                ),
            ));
        }

        let mut vals: Vec<f64> = Vec::with_capacity(args.len());
        let mut idx: Vec<usize> = Vec::with_capacity(args.len());
        for (k, kind) in sig.iter().enumerate() {
            match kind {
                ArgKind::Num => {
                    let e = expr::parse(&args[k]).map_err(|e| err(lineno, e.col, e.msg))?;
                    vals.push(e.eval(&params).map_err(|e| err(lineno, e.col, e.msg))?);
                    idx.push(usize::MAX);
                }
                ArgKind::Node => {
                    let n = ident(&args[k]).ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() argument {} must be a node name", call, k + 1),
                        )
                    })?;
                    let i = graph.find(&n).ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() references undefined node {n:?}", call),
                        )
                    })?;
                    idx.push(i);
                    vals.push(0.0);
                }
            }
        }
        let int_arg = |slot: usize, lo: i64, v: f64| -> Result<usize, ModelError> {
            if v.fract() != 0.0 || v < lo as f64 || v > 4_000_000.0 {
                return Err(err(
                    lineno,
                    1,
                    format!(
                        "{}() argument {} must be a whole number >= {lo} (got {v})",
                        call,
                        slot + 1
                    ),
                ));
            }
            Ok(v as usize)
        };
        let node = match call.as_str() {
            "box" => GeoNode::Box {
                w: vals[0],
                h: vals[1],
                d: vals[2],
            },
            "cylinder" => GeoNode::Cylinder {
                r: vals[0],
                h: vals[1],
                segs: int_arg(2, 3, vals[2])?,
            },
            "sphere" => GeoNode::Sphere {
                r: vals[0],
                rings: int_arg(1, 2, vals[1])?,
                segs: int_arg(2, 3, vals[2])?,
            },
            "move" => GeoNode::Move {
                node: idx[0],
                x: vals[1],
                y: vals[2],
                z: vals[3],
            },
            "union" => GeoNode::Union {
                a: idx[0],
                b: idx[1],
            },
            "cut" => GeoNode::Cut {
                a: idx[0],
                b: idx[1],
            },
            "intersect" => GeoNode::Intersect {
                a: idx[0],
                b: idx[1],
            },
            _ => unreachable!("signature matched above"),
        };
        graph.push(&name, node).map_err(|m| err(lineno, 1, m))?;
        node_count += 1;
    }

    if node_count == 0 {
        return Err(err(0, 0, "model defines no geometry nodes".to_string()));
    }
    Ok(Model {
        name: file.to_string(),
        density_g_cm3: density.unwrap_or(1.0),
        graph,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg;
    use crate::stl;

    const DEMO: &str = "
# bracket demo: parameters drive geometry through expressions
density = 7.85
param plate_w = 60
param plate_h = plate_w / 5          # 12
param bore_r  = plate_h / 2 - 1      # 5
plate = box(plate_w, plate_h, 40)
bore  = cylinder(bore_r, plate_h + 4, 64)
part  = cut(plate, bore)
";

    #[test]
    fn demo_file_parses_to_expected_graph() {
        let m = parse_model(DEMO, "demo.pcm").expect("demo must parse");
        assert_eq!(m.density_g_cm3, 7.85);
        assert_eq!(m.graph.nodes.len(), 3);
        assert_eq!(m.graph.root(), Some(2));
        assert_eq!(m.graph.find("bore"), Some(1));
        let GeoNode::Cut { a, b } = m.graph.nodes[2].1 else {
            panic!("last node must be the cut");
        };
        assert_eq!((a, b), (0, 1));
        let GeoNode::Cylinder { r, h, segs } = m.graph.nodes[1].1 else {
            panic!("bore must be a cylinder");
        };
        assert_eq!(
            (r, h, segs),
            (5.0, 16.0, 64),
            "expressions must chain params"
        );
    }

    #[test]
    fn demo_file_builds_and_mass_readout() {
        let m = parse_model(DEMO, "demo.pcm").unwrap();
        let s = csg::evaluate(&m.graph).unwrap();
        let inscribed = 0.5 * 64.0 * 25.0 * (std::f64::consts::TAU / 64.0).sin() * 12.0;
        assert!(
            (s.volume_mm3 - (28800.0 - inscribed)).abs() < 1e-9,
            "plate 60x12x40 = 28800 minus inscribed-64 prism"
        );
        assert_eq!(s.genus, 1);
        let mass = s.mass_g(m.density_g_cm3);
        assert!((mass - (s.volume_mm3 * 7.85 / 1000.0)).abs() < 1e-12);
        eprintln!(
            "demo: V={:.6} mm^3  mass={:.4} g  tris={}",
            s.volume_mm3, mass, s.num_tri
        );
    }

    #[test]
    fn rebuild_is_bit_stable() {
        let m = parse_model(DEMO, "demo.pcm").unwrap();
        let s1 = csg::evaluate(&m.graph).unwrap();
        let s2 = csg::evaluate(&m.graph).unwrap();
        assert_eq!(s1.volume_mm3.to_bits(), s2.volume_mm3.to_bits());
        assert_eq!(s1.mesh, s2.mesh);
        assert_eq!(stl::to_stl_binary(&s1.mesh), stl::to_stl_binary(&s2.mesh));
    }

    #[test]
    fn parse_errors_are_located() {
        let e = parse_model("part = box(1, 2, )", "x.pcm").unwrap_err();
        assert_eq!((e.line, e.file.as_str()), (1, "x.pcm"));
        let e = parse_model("a = box(1,2,3)\nb = union(c, a)", "x.pcm").unwrap_err();
        assert_eq!(e.line, 2);
        assert!(e.msg.contains("undefined node \"c\""), "{e}");
        let e = parse_model("a = box(1,2,3)\na = box(2,2,2)", "x.pcm").unwrap_err();
        assert!(e.msg.contains("defined twice"), "{e}");
        let e = parse_model("a = cylinder(1, 2, 3.5)", "x.pcm").unwrap_err();
        assert!(e.msg.contains("whole number"), "{e}");
        let e = parse_model("a = box(1,2,3)\nb = cut(a, box(1,1,1))", "x.pcm").unwrap_err();
        assert!(e.msg.contains("must be a node name"), "{e}");
        let e = parse_model("a = frobnicate(1)", "x.pcm").unwrap_err();
        assert!(e.msg.contains("unknown geometry call"), "{e}");
        let e = parse_model("# only comments", "x.pcm").unwrap_err();
        assert!(e.msg.contains("no geometry nodes"), "{e}");
        let e = parse_model("param a = 1 +\nb = box(a,1,1)", "x.pcm").unwrap_err();
        assert!(e.msg.contains("expression ends early"), "{e}");
        let e = parse_model("density = -2\na = box(1,1,1)", "x.pcm").unwrap_err();
        assert!(e.msg.contains("density"), "{e}");
        let e = parse_model("param r = 0\nb = sphere(r, 8, 8)\nc = box(1,1,1)", "x.pcm");
        // radius 0 survives parsing; the *kernel* must reject it.
        let m = e.unwrap();
        assert!(
            csg::evaluate(&m.graph).is_err(),
            "zero-radius sphere must fail at the kernel"
        );
    }

    #[test]
    fn comment_and_whitespace_robustness() {
        let src =
            "  # leading comment\n\n  a = box(1, 2, 3)   # inline\n  b = move ( a , 1 , 0 , 0 ) \n";
        let m = parse_model(src, "ws.pcm").unwrap();
        assert_eq!(m.graph.nodes.len(), 2);
    }
}
