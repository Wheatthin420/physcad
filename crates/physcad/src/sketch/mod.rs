//! Milestone 4: 2D sketch entities + geometric constraints, solved by our
//! own Levenberg-Marquardt (`solver`). slvs is GPL-3.0 (M1 survey) and
//! stays vendored-out without explicit user approval, so the solver here
//! is our own; 3BCAD (Apache-2.0) was studied for design, not linked.
//! All linear algebra is hand-rolled, all f64.
//!
//! The DOF model (FreeCAD-sketcher style, simpler):
//! - `Point` owns 2 params (x, y).
//! - `Circle` owns 1 param (radius); its center is a `Point` entity.
//! - `Line` and `Arc` own *no* params: they reference point entities, so
//!   endpoint sharing is structural (no coincident bookkeeping needed for
//!   the common case) and coincident remains available to merge points.
//!
//! Coordinates are millimetres in the sketch plane; constraint angles are
//! radians in the API (the .pcm file writes degrees).

pub mod loops;
pub mod solver;

use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum Entity {
    Point {
        x: f64,
        y: f64,
    },
    Line {
        a: usize,
        b: usize,
    },
    Circle {
        center: usize,
        radius: f64,
    },
    /// Center + start + end point entities; renders sweep CCW from start.
    Arc {
        center: usize,
        start: usize,
        end: usize,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub enum Constraint {
    /// Merge two point entities (2 equations).
    Coincident {
        a: usize,
        b: usize,
    },
    /// |pa - pb| = value (1 eq).
    Distance {
        a: usize,
        b: usize,
        value: f64,
    },
    /// Signed CCW angle from direction of line `a` to line `b`.
    Angle {
        a: usize,
        b: usize,
        radians: f64,
    },
    Horizontal {
        l: usize,
    },
    Vertical {
        l: usize,
    },
    /// Circle/arc radius = value (1 eq). For arcs the radius is derived
    /// from |start - center|.
    Radius {
        c: usize,
        value: f64,
    },
    /// Line tangent to circle: signed distance from center to line
    /// (either side allowed; solved as distance² = radius²).
    TangentLineCircle {
        line: usize,
        circle: usize,
    },
    /// Line tangent to arc, same convention: distance² = |start-center|².
    TangentLineArc {
        line: usize,
        arc: usize,
    },
    /// Two circles externally tangent: |c1 - c2| = r1 + r2.
    TangentCircles {
        a: usize,
        b: usize,
    },
    /// Pin a point entity to fixed coordinates (2 eqs).
    Fix {
        p: usize,
        x: f64,
        y: f64,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct NamedConstraint {
    pub name: String,
    pub kind: Constraint,
}

#[derive(Clone, Debug)]
pub struct SketchError(pub String);

impl fmt::Display for SketchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for SketchError {}

/// A sketch: ordered named entities + ordered named constraints.
/// Entity operands are indices into `entities` and always reference
/// *earlier* entities (declaration order = dependency order, matching
/// the GeoGraph convention from M3).
#[derive(Clone, Debug)]
pub struct Sketch {
    pub name: String,
    pub entities: Vec<(String, Entity)>,
    pub constraints: Vec<NamedConstraint>,
}

impl Sketch {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            entities: Vec::new(),
            constraints: Vec::new(),
        }
    }

    pub fn entity_index(&self, name: &str) -> Option<usize> {
        self.entities.iter().position(|(n, _)| n == name)
    }

    pub fn constraint_index(&self, name: &str) -> Option<usize> {
        self.constraints.iter().position(|c| c.name == name)
    }

    /// Add an entity; operands must exist already and names must be unique.
    pub fn add_entity(&mut self, name: &str, e: Entity) -> Result<usize, SketchError> {
        if self.entity_index(name).is_some() {
            return Err(SketchError(format!(
                "sketch {}: entity {name:?} defined twice",
                self.name
            )));
        }
        let idx = self.entities.len();
        for op in Self::entity_operands(&e) {
            if op >= idx {
                return Err(SketchError(format!(
                    "sketch {}: entity {name} references operand #{op} before definition",
                    self.name
                )));
            }
        }
        // Kind checks: Line/Arc endpoints and Circle centers must be points.
        let expect_point = |i: usize, role: &str| -> Result<(), SketchError> {
            match self.entities.get(i).map(|(_, e)| e) {
                Some(Entity::Point { .. }) => Ok(()),
                Some(other) => Err(SketchError(format!(
                    "sketch {}: entity {name}: {role} #{i} is {}, must be a point",
                    self.name,
                    entity_kind(other)
                ))),
                None => Err(SketchError(format!(
                    "sketch {}: entity {name}: {role} #{i} does not exist",
                    self.name
                ))),
            }
        };
        match &e {
            Entity::Line { a, b } => {
                expect_point(*a, "endpoint a")?;
                expect_point(*b, "endpoint b")?;
            }
            Entity::Circle { center, .. } => expect_point(*center, "center")?,
            Entity::Arc { center, start, end } => {
                expect_point(*center, "center")?;
                expect_point(*start, "start")?;
                expect_point(*end, "end")?;
            }
            Entity::Point { .. } => {}
        }
        self.entities.push((name.to_string(), e));
        Ok(idx)
    }

    /// Add a constraint; all entity operands must exist and match the
    /// kinds the constraint requires (diagnostics, never a panic).
    pub fn add_constraint(&mut self, name: &str, c: Constraint) -> Result<(), SketchError> {
        if self.constraint_index(name).is_some() {
            return Err(SketchError(format!(
                "sketch {}: constraint {name:?} defined twice",
                self.name
            )));
        }
        let need = |i: usize, what: &str, kinds: &[&str]| -> Result<(), SketchError> {
            match self.entities.get(i).map(|(_, e)| e) {
                Some(e) if kinds.contains(&entity_kind(e)) => Ok(()),
                Some(e) => Err(SketchError(format!(
                    "sketch {}: constraint {name}: {what} #{i} is {}, must be {}",
                    self.name,
                    entity_kind(e),
                    kinds.join(" or ")
                ))),
                None => Err(SketchError(format!(
                    "sketch {}: constraint {name}: {what} #{i} does not exist",
                    self.name
                ))),
            }
        };
        const PT: &[&str] = &["point"];
        const LN: &[&str] = &["line"];
        const CR: &[&str] = &["circle"];
        const ARC: &[&str] = &["arc"];
        match &c {
            Constraint::Coincident { a, b } | Constraint::Distance { a, b, .. } => {
                need(*a, "operand a", PT)?;
                need(*b, "operand b", PT)?;
            }
            Constraint::Angle { a, b, .. } => {
                need(*a, "operand a", LN)?;
                need(*b, "operand b", LN)?;
            }
            Constraint::Horizontal { l } | Constraint::Vertical { l } => need(*l, "line", LN)?,
            Constraint::Radius { c, .. } => need(*c, "circle-or-arc", &["circle", "arc"])?,
            Constraint::TangentLineCircle { line, circle } => {
                need(*line, "line", LN)?;
                need(*circle, "circle", CR)?;
            }
            Constraint::TangentLineArc { line, arc } => {
                need(*line, "line", LN)?;
                need(*arc, "arc", ARC)?;
            }
            Constraint::TangentCircles { a, b } => {
                need(*a, "circle a", CR)?;
                need(*b, "circle b", CR)?;
            }
            Constraint::Fix { p, .. } => need(*p, "point", PT)?,
        }
        self.constraints.push(NamedConstraint {
            name: name.to_string(),
            kind: c,
        });
        Ok(())
    }

    fn entity_operands(e: &Entity) -> Vec<usize> {
        match e {
            Entity::Point { .. } => vec![],
            Entity::Line { a, b } => vec![*a, *b],
            Entity::Circle { center, .. } => vec![*center],
            Entity::Arc { center, start, end } => vec![*center, *start, *end],
        }
    }

    /// Total scalar DOF: point 2, circle 1 (radius), line/arc 0.
    pub fn n_params(&self) -> usize {
        self.entities
            .iter()
            .map(|(_, e)| match e {
                Entity::Point { .. } => 2,
                Entity::Circle { .. } => 1,
                _ => 0,
            })
            .sum()
    }

    /// Global index of the first param owned by entity `i`.
    pub fn entity_dof(&self, i: usize) -> usize {
        self.entities[..i]
            .iter()
            .map(|(_, e)| match e {
                Entity::Point { .. } => 2,
                Entity::Circle { .. } => 1,
                _ => 0,
            })
            .sum()
    }

    /// Flattened parameter vector in DOF-layout order (x0 y0 x1 y1 ... plus
    /// one radius slot after the center point of each circle).
    pub fn params_vec(&self) -> Vec<f64> {
        let mut v = Vec::with_capacity(self.n_params());
        for (_, e) in &self.entities {
            match e {
                Entity::Point { x, y } => {
                    v.push(*x);
                    v.push(*y);
                }
                Entity::Circle { radius, .. } => v.push(*radius),
                _ => {}
            }
        }
        v
    }

    /// Write a solved parameter vector back into a copy of this sketch.
    pub fn with_solution(&self, p: &[f64]) -> Sketch {
        assert_eq!(p.len(), self.n_params(), "solution length mismatch");
        let mut out = self.clone();
        let mut k = 0usize;
        for (_, e) in &mut out.entities {
            match e {
                Entity::Point { x, y } => {
                    *x = p[k];
                    *y = p[k + 1];
                    k += 2;
                }
                Entity::Circle { radius, .. } => {
                    *radius = p[k];
                    k += 1;
                }
                _ => {}
            }
        }
        out
    }

    /// Global param indices touched by a constraint, in the slot order that
    /// `eval_term` (solver) expects. This function and the term evaluators
    /// are the *only* two places that define the layout.
    pub fn term_params(&self, c: &Constraint) -> Vec<usize> {
        let mut v = Vec::new();
        let point = |i: usize, v: &mut Vec<usize>| {
            let d = self.entity_dof(i);
            v.extend_from_slice(&[d, d + 1]);
        };
        let line = |i: usize, v: &mut Vec<usize>| {
            if let Entity::Line { a, b } = &self.entities[i].1 {
                point(*a, v);
                point(*b, v);
            }
        };
        let circle = |i: usize, v: &mut Vec<usize>| {
            if let Entity::Circle { center, .. } = &self.entities[i].1 {
                point(*center, v);
                v.push(self.entity_dof(i));
            }
        };
        let arc = |i: usize, v: &mut Vec<usize>| {
            if let Entity::Arc { center, start, .. } = &self.entities[i].1 {
                point(*center, v);
                point(*start, v);
            }
        };
        match c {
            Constraint::Coincident { a, b } | Constraint::Distance { a, b, .. } => {
                point(*a, &mut v);
                point(*b, &mut v);
            }
            Constraint::Angle { a, b, .. } => {
                line(*a, &mut v);
                line(*b, &mut v);
            }
            Constraint::Horizontal { l } | Constraint::Vertical { l } => line(*l, &mut v),
            Constraint::Radius { c, .. } => v.push(self.entity_dof(*c)),
            Constraint::TangentLineCircle {
                line: l,
                circle: ci,
            } => {
                line(*l, &mut v);
                circle(*ci, &mut v);
            }
            Constraint::TangentLineArc { line: l, arc: a } => {
                line(*l, &mut v);
                arc(*a, &mut v);
            }
            Constraint::TangentCircles { a, b } => {
                circle(*a, &mut v);
                circle(*b, &mut v);
            }
            Constraint::Fix { p, .. } => point(*p, &mut v),
        }
        v
    }

    /// Equation count of a constraint (rows it contributes to the Jacobian).
    pub fn term_equations(c: &Constraint) -> usize {
        match c {
            Constraint::Coincident { .. } | Constraint::Fix { .. } => 2,
            _ => 1,
        }
    }

    /// Polyline approximation of every entity, for rendering/debug export.
    /// Arcs sweep CCW from start to end; segment counts are deterministic
    /// (max 7.5 deg per segment, circles 48-gons).
    pub fn polylines(&self, chord_tol_mm: f64) -> Vec<Vec<[f64; 2]>> {
        let pt = |i: usize| match self.entities[i].1 {
            Entity::Point { x, y } => [x, y],
            _ => unreachable!("entity kind checked at add time"),
        };
        let mut out = Vec::new();
        for (_, e) in &self.entities {
            match e {
                Entity::Point { x, y } => out.push(vec![[*x, *y]]),
                Entity::Line { a, b } => out.push(vec![pt(*a), pt(*b)]),
                Entity::Circle { center, radius } => {
                    let c = pt(*center);
                    out.push(sample_arc(
                        c,
                        *radius,
                        0.0,
                        std::f64::consts::TAU,
                        chord_tol_mm,
                    ));
                }
                Entity::Arc { center, start, end } => {
                    let c = pt(*center);
                    let s = pt(*start);
                    let en = pt(*end);
                    let r = ((s[0] - c[0]).powi(2) + (s[1] - c[1]).powi(2)).sqrt();
                    let a0 = (s[1] - c[1]).atan2(s[0] - c[0]);
                    let a1 = (en[1] - c[1]).atan2(en[0] - c[0]);
                    let mut sweep = a1 - a0;
                    while sweep <= 0.0 {
                        sweep += std::f64::consts::TAU;
                    }
                    out.push(sample_arc(c, r, a0, a0 + sweep, chord_tol_mm));
                }
            }
        }
        out
    }
}

pub fn entity_kind(e: &Entity) -> &'static str {
    match e {
        Entity::Point { .. } => "point",
        Entity::Line { .. } => "line",
        Entity::Circle { .. } => "circle",
        Entity::Arc { .. } => "arc",
    }
}

fn sample_arc(c: [f64; 2], r: f64, a0: f64, a1: f64, chord_tol_mm: f64) -> Vec<[f64; 2]> {
    let sweep = a1 - a0;
    // chord sagitta s = r(1-cos(dθ/2)) <= tol  =>  dθ <= 2 acos(1-tol/r)
    let dmax = if chord_tol_mm >= r {
        std::f64::consts::PI / 24.0 // tiny circle: fall back to 7.5-deg steps
    } else {
        2.0 * (1.0 - chord_tol_mm / r).acos()
    };
    let segs = (sweep.abs() / dmax).ceil().max(1.0) as usize;
    let d = sweep / segs as f64;
    (0..=segs)
        .map(|k| {
            let a = a0 + d * k as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect()
}
