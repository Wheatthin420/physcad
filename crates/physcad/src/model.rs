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
//!   NAME = torus(RING_R, TUBE_R, MAJOR_SEGS, MINOR_SEGS)
//!   NAME = move(NODE, X, Y, Z)
//!   NAME = union(A, B) | cut(A, B) | intersect(A, B)
//!
//!   # --- M4: sketches (entities + constraints; solved on demand) ---
//!   NAME = newsketch()
//!   NAME = point(SKETCH, X, Y)
//!   NAME = line(SKETCH, P1, P2)          # endpoints are point entities
//!   NAME = circle(SKETCH, CENTER, R)
//!   NAME = arc(SKETCH, CENTER, START, END)
//!   NAME = coincident(P1, P2) | distance(P1, P2, D) | angle(L1, L2, DEG)
//!   NAME = horizontal(L) | vertical(L) | radius(C, R) | fix(P, X, Y)
//!   NAME = tangent(L, C) | tangent(L, ARC) | tangent(C1, C2)
//!
//!   # --- M5: sketch regions -> solids (compose with CSG like any node) ---
//!   NAME = extrude(SKETCH, H)                 # solid z in [0, H]
//!   NAME = extrude(SKETCH, H, OFF)            # solid z in [OFF, OFF+H]
//!   NAME = extrude(SKETCH, H, OFF, SEGS)      # arcs/circles sampled as
//!                                             # SEGS-gons (0/omitted = chord tol)
//!   NAME = revolve(SKETCH, AXIS_LINE)         # full 360 deg about in-plane line
//!   NAME = revolve(SKETCH, AXIS_LINE, DEG)    # partial sweep (0 < DEG <= 360)
//!   NAME = revolve(SKETCH, AX, DEG, SWEEP_SEGS[, PROFILE_SEGS])
//! ```
//!
//! The root of the solid is the LAST node defined. Units: millimetres.
//! Node operands are plain names and must already be defined (files read
//! top-to-bottom — declaration order is the dependency order, typed as a
//! `Timeline` of features instead of parsed at draw time).
//!
//! M6: a geometry statement may PIN its feature id — `NAME@K = call(...)`
//! — making that feature's identity stable across renames of *other*
//! features and across insertion of new features mid-file. Unpinned
//! features take the next monotonic id at parse time (safe for appends;
//! insertion BEFORE an unpinned feature renumbers it — the documented
//! failure mode FM-6, why the editor must always pin).

use std::collections::BTreeMap;
use std::fmt;

use crate::expr;
use crate::sketch::{Constraint, Entity, Sketch};
use crate::timeline::{FeatureId, GeoNode, Timeline};

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
    pub graph: Timeline,
    /// M4: sketches declared in the same file (solved on demand).
    pub sketches: Vec<Sketch>,
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
    if inner.trim().is_empty() {
        return Ok((head.to_string(), Vec::new()));
    }
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
    /// M5: a declared sketch name (resolved against the sketch table).
    Sketch,
    /// M5: a declared sketch entity name (resolved against ent_names).
    SketchEntity,
}

/// True if `call` is one of the M4 sketch statements (handled outside the
/// geometry-node path).
fn is_sketch_call(call: &str) -> bool {
    matches!(
        call,
        "newsketch"
            | "point"
            | "line"
            | "circle"
            | "arc"
            | "coincident"
            | "distance"
            | "angle"
            | "horizontal"
            | "vertical"
            | "radius"
            | "tangent"
            | "fix"
    )
}

fn sketch_arg_ident(
    arg: &str,
    file: &str,
    lineno: usize,
    what: &str,
) -> Result<String, ModelError> {
    ident(arg).ok_or_else(|| ModelError {
        file: file.to_string(),
        line: lineno,
        col: 1,
        msg: format!("{what}: operand {arg:?} is not a plain identifier"),
    })
}

/// Apply one sketch statement to the model's sketch list. Entity names
/// share the global namespace with geometry nodes (duplicate = error).
#[allow(clippy::too_many_arguments)] // parser context bundle, not an API
fn apply_sketch_call(
    call: &str,
    name: &str,
    args: &[String],
    lineno: usize,
    file: &str,
    params: &BTreeMap<String, f64>,
    graph: &Timeline,
    sketches: &mut Vec<Sketch>,
    ent_names: &mut BTreeMap<String, (usize, usize)>,
) -> Result<(), ModelError> {
    let err = |col: usize, msg: String| ModelError {
        file: file.to_string(),
        line: lineno,
        col,
        msg,
    };
    let argc = |want: usize| -> Result<(), ModelError> {
        if args.len() != want {
            return Err(err(
                1,
                format!("{call}() takes {want} arguments, got {}", args.len()),
            ));
        }
        Ok(())
    };
    let num = |s: &str| -> Result<f64, ModelError> {
        let e = expr::parse(s).map_err(|e| err(e.col, e.msg))?;
        let v = e.eval(params).map_err(|e| err(e.col, e.msg))?;
        if !v.is_finite() {
            return Err(err(1, format!("{call} {name}: value is not finite")));
        }
        Ok(v)
    };
    let find_sk = |n: &str| -> Result<usize, ModelError> {
        let n = sketch_arg_ident(n, file, lineno, call)?;
        sketches.iter().position(|s| s.name == n).ok_or_else(|| {
            err(
                1,
                format!("{call}: {n:?} is not a sketch (declare `NAME = newsketch()` above)"),
            )
        })
    };
    let find_ent = |n: &str| -> Result<(usize, usize), ModelError> {
        let n = sketch_arg_ident(n, file, lineno, call)?;
        ent_names.get(&n).copied().ok_or_else(|| {
            err(
                1,
                format!("{call}: entity {n:?} is not defined (entities must be declared above)"),
            )
        })
    };
    if call != "newsketch"
        && (graph.find(name).is_some()
            || ent_names.contains_key(name)
            || sketches.iter().any(|s| s.name == name))
    {
        return Err(err(1, format!("name {name:?} defined twice")));
    }
    match call {
        "newsketch" => {
            argc(0)?;
            if sketches.iter().any(|s| s.name == name) {
                return Err(err(1, format!("sketch {name:?} defined twice")));
            }
            sketches.push(Sketch::new(name));
        }
        "point" => {
            argc(3)?;
            let ski = find_sk(&args[0])?;
            let (x, y) = (num(&args[1])?, num(&args[2])?);
            let idx = sketches[ski]
                .add_entity(name, Entity::Point { x, y })
                .map_err(|e| err(1, e.to_string()))?;
            ent_names.insert(name.to_string(), (ski, idx));
        }
        "line" => {
            argc(3)?;
            let ski = find_sk(&args[0])?;
            let (sa, ia) = find_ent(&args[1])?;
            let (sb, ib) = find_ent(&args[2])?;
            same_sketch(sketches, call, ski, sa, sb, file, lineno)?;
            let idx = sketches[ski]
                .add_entity(name, Entity::Line { a: ia, b: ib })
                .map_err(|e| err(1, e.to_string()))?;
            ent_names.insert(name.to_string(), (ski, idx));
        }
        "circle" => {
            argc(3)?;
            let ski = find_sk(&args[0])?;
            let (sc, ic) = find_ent(&args[1])?;
            same_sketch(sketches, call, ski, sc, sc, file, lineno)?;
            let r = num(&args[2])?;
            let idx = sketches[ski]
                .add_entity(
                    name,
                    Entity::Circle {
                        center: ic,
                        radius: r,
                    },
                )
                .map_err(|e| err(1, e.to_string()))?;
            ent_names.insert(name.to_string(), (ski, idx));
        }
        "arc" => {
            argc(4)?;
            let ski = find_sk(&args[0])?;
            let (sc, ic) = find_ent(&args[1])?;
            let (ss, is_) = find_ent(&args[2])?;
            let (se, ie) = find_ent(&args[3])?;
            same_sketch(sketches, call, ski, sc, ss, file, lineno)?;
            same_sketch(sketches, call, ski, sc, se, file, lineno)?;
            let idx = sketches[ski]
                .add_entity(
                    name,
                    Entity::Arc {
                        center: ic,
                        start: is_,
                        end: ie,
                    },
                )
                .map_err(|e| err(1, e.to_string()))?;
            ent_names.insert(name.to_string(), (ski, idx));
        }
        "coincident" => {
            argc(2)?;
            let (sa, ia) = find_ent(&args[0])?;
            let (sb, ib) = find_ent(&args[1])?;
            same_sketch(sketches, call, sa, sa, sb, file, lineno)?;
            sketches[sa]
                .add_constraint(name, Constraint::Coincident { a: ia, b: ib })
                .map_err(|e| err(1, e.to_string()))?;
        }
        "distance" => {
            argc(3)?;
            let (sa, ia) = find_ent(&args[0])?;
            let (sb, ib) = find_ent(&args[1])?;
            same_sketch(sketches, call, sa, sa, sb, file, lineno)?;
            let v = num(&args[2])?;
            sketches[sa]
                .add_constraint(
                    name,
                    Constraint::Distance {
                        a: ia,
                        b: ib,
                        value: v,
                    },
                )
                .map_err(|e| err(1, e.to_string()))?;
        }
        "angle" => {
            argc(3)?;
            let (sa, ia) = find_ent(&args[0])?;
            let (sb, ib) = find_ent(&args[1])?;
            same_sketch(sketches, call, sa, sa, sb, file, lineno)?;
            let deg = num(&args[2])?;
            sketches[sa]
                .add_constraint(
                    name,
                    Constraint::Angle {
                        a: ia,
                        b: ib,
                        radians: deg.to_radians(),
                    },
                )
                .map_err(|e| err(1, e.to_string()))?;
        }
        "horizontal" | "vertical" => {
            argc(1)?;
            let (sa, ia) = find_ent(&args[0])?;
            let c = if call == "horizontal" {
                Constraint::Horizontal { l: ia }
            } else {
                Constraint::Vertical { l: ia }
            };
            sketches[sa]
                .add_constraint(name, c)
                .map_err(|e| err(1, e.to_string()))?;
        }
        "radius" => {
            argc(2)?;
            let (sa, ia) = find_ent(&args[0])?;
            let v = num(&args[1])?;
            sketches[sa]
                .add_constraint(name, Constraint::Radius { c: ia, value: v })
                .map_err(|e| err(1, e.to_string()))?;
        }
        "tangent" => {
            argc(2)?;
            let (sa, ia) = find_ent(&args[0])?;
            let (sb, ib) = find_ent(&args[1])?;
            same_sketch(sketches, call, sa, sa, sb, file, lineno)?;
            // classify by entity kinds; order-insensitive pair match
            let ka = entity_kind_name(sketches, sa, ia);
            let kb = entity_kind_name(sketches, sb, ib);
            let c = if ka == "line" || kb == "line" {
                let (li, ci) = if ka == "line" { (ia, ib) } else { (ib, ia) };
                let ck = if ka == "line" { kb } else { ka };
                match ck {
                    "circle" => Constraint::TangentLineCircle {
                        line: li,
                        circle: ci,
                    },
                    "arc" => Constraint::TangentLineArc { line: li, arc: ci },
                    other => {
                        return Err(err(
                            1,
                            format!(
                                "tangent(line, {other}): use line+circle, line+arc or circle+circle"
                            ),
                        ));
                    }
                }
            } else if ka == "circle" && kb == "circle" {
                Constraint::TangentCircles { a: ia, b: ib }
            } else {
                return Err(err(
                    1,
                    format!("tangent({ka}, {kb}): use line+circle, line+arc or circle+circle"),
                ));
            };
            sketches[sa]
                .add_constraint(name, c)
                .map_err(|e| err(1, e.to_string()))?;
        }
        "fix" => {
            argc(3)?;
            let (sa, ia) = find_ent(&args[0])?;
            let (x, y) = (num(&args[1])?, num(&args[2])?);
            sketches[sa]
                .add_constraint(name, Constraint::Fix { p: ia, x, y })
                .map_err(|e| err(1, e.to_string()))?;
        }
        _ => return Err(err(1, format!("unknown sketch call {call:?}"))),
    }
    Ok(())
}

fn same_sketch(
    sketches: &[Sketch],
    call: &str,
    want: usize,
    a: usize,
    b: usize,
    file: &str,
    lineno: usize,
) -> Result<(), ModelError> {
    let off = |i: usize| ModelError {
        file: file.to_string(),
        line: lineno,
        col: 1,
        msg: format!(
            "{call}: operand is in sketch {:?}, expected {:?}",
            sketches[i].name, sketches[want].name
        ),
    };
    if a != want {
        return Err(off(a));
    }
    if b != want {
        return Err(off(b));
    }
    Ok(())
}

fn entity_kind_name(sketches: &[Sketch], ski: usize, ent: usize) -> &'static str {
    crate::sketch::entity_kind(&sketches[ski].entities[ent].1)
}

/// Parse model source into parameters + a GeoGraph (no kernel work yet).
pub fn parse_model(src: &str, file: &str) -> Result<Model, ModelError> {
    let mut graph = Timeline::default();
    let mut params: BTreeMap<String, f64> = BTreeMap::new();
    let mut density: Option<f64> = None;
    let mut sketches: Vec<Sketch> = Vec::new();
    let mut ent_names: BTreeMap<String, (usize, usize)> = BTreeMap::new();
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

        // M6: optional feature-id pin `NAME@K` on geometry statements.
        let (nm, pin) = match lhs.split_once('@') {
            Some((n, k)) => {
                let Ok(k) = k.trim().parse::<u32>() else {
                    return Err(err(
                        lineno,
                        1,
                        format!("feature id {k:?} must be an integer (NAME@K, K >= 1)"),
                    ));
                };
                (n, Some(FeatureId(k)))
            }
            None => (lhs, None),
        };
        let Some(name) = ident(nm) else {
            return Err(err(lineno, 1, format!("bad node name {lhs:?}")));
        };
        let (call, args) = parse_call(rhs).map_err(|(col, m)| err(lineno, eq + 1 + col, m))?;

        // M4 sketch statements live in their own tables, not the timeline.
        if is_sketch_call(&call) {
            if pin.is_some() {
                return Err(err(
                    lineno,
                    1,
                    "feature id pin (@K) is only allowed on geometry nodes".to_string(),
                ));
            }
            apply_sketch_call(
                &call,
                &name,
                &args,
                lineno,
                file,
                &params,
                &graph,
                &mut sketches,
                &mut ent_names,
            )?;
            continue;
        }

        // Per-call argument signature: numeric expressions vs node names
        // vs (M5) sketch/entity names. Evaluating a node-name slot as an
        // expression would surface the wrong error ("unknown variable"),
        // so classify before evaluating. extrude/revolve have optional
        // trailing args, hence arity-keyed signatures.
        let sig: Vec<ArgKind> = match call.as_str() {
            "box" | "cylinder" | "sphere" => vec![ArgKind::Num; 3],
            "torus" => vec![ArgKind::Num; 4],
            "move" => vec![ArgKind::Node, ArgKind::Num, ArgKind::Num, ArgKind::Num],
            "union" | "cut" | "intersect" => vec![ArgKind::Node; 2],
            "extrude" => match args.len() {
                2 => vec![ArgKind::Sketch, ArgKind::Num],
                3 => vec![ArgKind::Sketch, ArgKind::Num, ArgKind::Num],
                4 => vec![ArgKind::Sketch, ArgKind::Num, ArgKind::Num, ArgKind::Num],
                n => {
                    return Err(err(
                        lineno,
                        1,
                        format!("extrude() takes 2..4 arguments, got {n}"),
                    ));
                }
            },
            "revolve" => match args.len() {
                2 => vec![ArgKind::Sketch, ArgKind::SketchEntity],
                3 => vec![ArgKind::Sketch, ArgKind::SketchEntity, ArgKind::Num],
                4 => vec![
                    ArgKind::Sketch,
                    ArgKind::SketchEntity,
                    ArgKind::Num,
                    ArgKind::Num,
                ],
                5 => vec![
                    ArgKind::Sketch,
                    ArgKind::SketchEntity,
                    ArgKind::Num,
                    ArgKind::Num,
                    ArgKind::Num,
                ],
                n => {
                    return Err(err(
                        lineno,
                        1,
                        format!("revolve() takes 2..5 arguments, got {n}"),
                    ));
                }
            },
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
        let mut names: Vec<String> = Vec::with_capacity(args.len());
        // M6: node-name slots carry stable FeatureIds (sketch/entity
        // slots keep FeatureId(0) and are never read for operands).
        let mut ops: Vec<FeatureId> = Vec::with_capacity(args.len());
        for (k, kind) in sig.iter().enumerate() {
            match kind {
                ArgKind::Num => {
                    let e = expr::parse(&args[k]).map_err(|e| err(lineno, e.col, e.msg))?;
                    vals.push(e.eval(&params).map_err(|e| err(lineno, e.col, e.msg))?);
                    idx.push(usize::MAX);
                    names.push(String::new());
                    ops.push(FeatureId(0));
                }
                ArgKind::Node => {
                    let n = ident(&args[k]).ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() argument {} must be a node name", call, k + 1),
                        )
                    })?;
                    let fid = graph.find(&n).ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() references undefined node {n:?}", call),
                        )
                    })?;
                    ops.push(fid);
                    idx.push(usize::MAX);
                    vals.push(0.0);
                    names.push(n);
                }
                ArgKind::Sketch => {
                    ops.push(FeatureId(0));
                    let n = ident(&args[k]).ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() argument {} must be a sketch name", call, k + 1),
                        )
                    })?;
                    let si = sketches.iter().position(|s| s.name == n).ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() references undefined sketch {n:?}", call),
                        )
                    })?;
                    idx.push(si);
                    vals.push(0.0);
                    names.push(n);
                }
                ArgKind::SketchEntity => {
                    ops.push(FeatureId(0));
                    let n = ident(&args[k]).ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() argument {} must be a sketch entity name", call, k + 1),
                        )
                    })?;
                    let (ski, ei) = ent_names.get(&n).copied().ok_or_else(|| {
                        err(
                            lineno,
                            1,
                            format!("{}() references undefined sketch entity {n:?}", call),
                        )
                    })?;
                    let kind_name = crate::sketch::entity_kind(&sketches[ski].entities[ei].1);
                    if kind_name != "line" {
                        return Err(err(
                            lineno,
                            1,
                            format!(
                                "{}() axis {n:?} is {kind_name}, must be a line entity",
                                call
                            ),
                        ));
                    }
                    idx.push(ei);
                    vals.push(0.0);
                    names.push(n);
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
            "torus" => GeoNode::Torus {
                r: vals[0],
                tube: vals[1],
                major_segs: int_arg(2, 3, vals[2])?,
                minor_segs: int_arg(3, 3, vals[3])?,
            },
            "extrude" => GeoNode::Extrude {
                sketch: names[0].clone(),
                height: vals[1],
                offset: if args.len() >= 3 { vals[2] } else { 0.0 },
                profile_segs: if args.len() >= 4 {
                    int_arg(3, 3, vals[3])?
                } else {
                    0
                },
            },
            "revolve" => {
                let deg = if args.len() >= 3 { vals[2] } else { 360.0 };
                if !(deg > 0.0 && deg <= 360.0) {
                    return Err(err(
                        lineno,
                        1,
                        format!("revolve() sweep angle must be in (0, 360] degrees (got {deg})"),
                    ));
                }
                GeoNode::Revolve {
                    sketch: names[0].clone(),
                    axis: names[1].clone(),
                    angle: deg.to_radians(),
                    sweep_segs: if args.len() >= 4 {
                        int_arg(3, 3, vals[3])?
                    } else {
                        96
                    },
                    profile_segs: if args.len() >= 5 {
                        int_arg(4, 3, vals[4])?
                    } else {
                        0
                    },
                }
            }
            "move" => GeoNode::Move {
                node: ops[0],
                x: vals[1],
                y: vals[2],
                z: vals[3],
            },
            "union" => GeoNode::Union {
                a: ops[0],
                b: ops[1],
            },
            "cut" => GeoNode::Cut {
                a: ops[0],
                b: ops[1],
            },
            "intersect" => GeoNode::Intersect {
                a: ops[0],
                b: ops[1],
            },
            _ => unreachable!("signature matched above"),
        };
        match pin {
            Some(id) => {
                graph
                    .push_with_id(&name, id, node)
                    .map_err(|m| err(lineno, 1, m))?;
            }
            None => {
                graph.push(&name, node).map_err(|m| err(lineno, 1, m))?;
            }
        }
        node_count += 1;
    }

    if node_count == 0 && sketches.is_empty() {
        return Err(err(
            0,
            0,
            "model defines no geometry or sketches".to_string(),
        ));
    }
    Ok(Model {
        name: file.to_string(),
        density_g_cm3: density.unwrap_or(1.0),
        graph,
        sketches,
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
        assert_eq!(m.graph.features.len(), 3);
        assert_eq!(m.graph.root(), Some(FeatureId(3)));
        assert_eq!(m.graph.find("bore"), Some(FeatureId(2)));
        let GeoNode::Cut { a, b } = m.graph.features[2].op else {
            panic!("last node must be the cut");
        };
        assert_eq!((a, b), (FeatureId(1), FeatureId(2)));
        let GeoNode::Cylinder { r, h, segs } = m.graph.features[1].op else {
            panic!("bore must be a cylinder");
        };
        assert_eq!(
            (r, h, segs),
            (5.0, 16.0, 64),
            "expressions must chain params"
        );
    }

    /// torus(RING, TUBE, MAJOR, MINOR): a major/minor swap or argument
    /// drift in the wiring must be test-visible. Pinned 2026-11-24 after
    /// a gate red run whose "2^21 tris" figure made a resolution-parameter
    /// bug the prime suspect (it wasn't one — but the guard now exists).
    #[test]
    fn torus_segment_args_wire_in_order() {
        let m = parse_model("ring = torus(1.0, 0.25, 48, 32)", "t.pcm").unwrap();
        let GeoNode::Torus {
            r,
            tube,
            major_segs,
            minor_segs,
        } = m.graph.features[0].op
        else {
            panic!("ring must be a torus");
        };
        assert_eq!((r, tube, major_segs, minor_segs), (1.0, 0.25, 48, 32));
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
        assert!(e.msg.contains("no geometry or sketches"), "{e}");
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
        assert_eq!(m.graph.features.len(), 2);
    }

    const SKETCH_DEMO: &str = include_str!("../../../examples/sketch-bracket.pcm");

    #[test]
    fn sketch_file_parses_and_solves_exact() {
        use crate::sketch::solver::{SolveStatus, solve};
        let m = parse_model(SKETCH_DEMO, "examples/sketch-bracket.pcm").unwrap();
        assert_eq!(m.sketches.len(), 1);
        assert_eq!(m.graph.features.len(), 1, "the plate box stays the root");
        let sk = &m.sketches[0];
        assert_eq!(sk.name, "br");
        assert_eq!(sk.entities.len(), 14);
        assert_eq!(sk.constraints.len(), 13);
        let (solved, rep) = solve(sk);
        eprintln!(
            "sketch-bracket: {rep} | max|res| {:.3e} l2 {:.3e} iters {}",
            rep.max_residual, rep.l2_residual, rep.iterations
        );
        assert_eq!(rep.status, SolveStatus::Solved);
        assert!(rep.max_residual < 1e-9, "residual {}", rep.max_residual);
        assert_eq!(rep.dof, 0);
        assert!(rep.redundant.is_empty());
        // fillet center must land at (w - fil, fil) = (68, 12)
        let ci = sk.entity_index("fillet").unwrap();
        let oc = match solved.entities[ci].1 {
            crate::sketch::Entity::Circle { center, radius } => (center, radius),
            _ => panic!(),
        };
        let (cx, cy) = match solved.entities[oc.0].1 {
            crate::sketch::Entity::Point { x, y } => (x, y),
            _ => panic!(),
        };
        eprintln!("fillet center ({cx:.12}, {cy:.12}) r {:.12}", oc.1);
        assert!((cx - 68.0).abs() < 1e-9 && (cy - 12.0).abs() < 1e-9);
        assert!((oc.1 - 12.0).abs() < 1e-9);
    }

    #[test]
    fn sketch_parse_errors_are_located_and_specific() {
        // constraint before its entity exists
        let e = parse_model("s = newsketch()\nd = distance(p, q, 5)", "x.pcm").unwrap_err();
        assert_eq!(e.line, 2);
        assert!(e.msg.contains("entity \"p\" is not defined"), "{e}");
        // wrong entity kind for the constraint
        let e = parse_model(
            "s = newsketch()\na = point(s,0,0)\nb = point(s,1,0)\nl = line(s,a,b)\nh = horizontal(a)",
            "x.pcm",
        )
        .unwrap_err();
        assert!(e.msg.contains("must be line"), "{e}");
        // tangent with an unsupported pair
        let e = parse_model(
            "s = newsketch()\na = point(s,0,0)\nb = point(s,1,0)\nt = tangent(a,b)",
            "x.pcm",
        )
        .unwrap_err();
        assert!(e.msg.contains("tangent(point, point)"), "{e}");
        // cross-sketch constraint
        let e = parse_model(
            "s1 = newsketch()\ns2 = newsketch()\na = point(s1,0,0)\nb = point(s2,1,0)\nd = distance(a,b,1)",
            "x.pcm",
        )
        .unwrap_err();
        assert!(e.msg.contains("expected \"s1\""), "{e}");
        // name clash with a geometry node
        let e = parse_model(
            "part = box(1,2,3)\ns = newsketch()\npart = point(s,0,0)",
            "x.pcm",
        )
        .unwrap_err();
        assert!(e.msg.contains("defined twice"), "{e}");
        // entity attached to something that is not a sketch
        let e = parse_model("part = box(1,2,3)\np = point(part,0,0)", "x.pcm").unwrap_err();
        assert!(e.msg.contains("not a sketch"), "{e}");
        // sketch-only file is legal now
        let m = parse_model("s = newsketch()\na = point(s, 1, 2)", "only.pcm").unwrap();
        assert_eq!(m.sketches.len(), 1);
        assert!(m.graph.features.is_empty());
        assert!(m.graph.root().is_none());
    }

    /// M6: `NAME@K` pins a stable feature id; unpinned features take the
    /// next monotonic id. Bad pins must be located errors, never panic.
    #[test]
    fn feature_id_pins_parse_and_validate() {
        let m = parse_model(
            "plate@5 = box(1,1,1)\nbore@9 = cylinder(0.5,2,8)\nlate = box(1,1,1)",
            "pin.pcm",
        )
        .unwrap();
        assert_eq!(m.graph.find("plate"), Some(FeatureId(5)));
        assert_eq!(m.graph.find("bore"), Some(FeatureId(9)));
        assert_eq!(m.graph.find("late"), Some(FeatureId(10))); // next after max pin
        assert!(matches!(m.graph.features[2].op, GeoNode::Box { .. }));
        let e = parse_model("a@1 = box(1,1,1)\nb@1 = box(1,1,1)", "p.pcm").unwrap_err();
        assert!(e.msg.contains("already used"), "{e}");
        let e = parse_model("a@0 = box(1,1,1)", "p.pcm").unwrap_err();
        assert!(e.msg.contains("invalid"), "{e}");
        let e = parse_model("a@x = box(1,1,1)", "p.pcm").unwrap_err();
        assert!(e.msg.contains("must be an integer"), "{e}");
        let e = parse_model("s@2 = newsketch()", "p.pcm").unwrap_err();
        assert!(e.msg.contains("only allowed on geometry nodes"), "{e}");
        // pinned operands resolve by name to the pinned id
        let m = parse_model("a@8 = box(1,1,1)\nb@3 = move(a, 1, 0, 0)", "p.pcm").unwrap();
        let GeoNode::Move { node, .. } = m.graph.features[1].op else {
            panic!("b must be a move");
        };
        assert_eq!(node, FeatureId(8));
    }
}
