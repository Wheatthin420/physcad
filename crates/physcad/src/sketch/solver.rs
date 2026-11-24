//! Levenberg-Marquardt constraint solver over `Sketch` (all f64, dense,
//! deterministic). Design notes (studied 3bcad's solver — its repo is
//! Apache-2.0 and its nalgebra dep is Apache-2.0/BSD-3 per crates.io,
//! verified 2026-11-24 — but we hand-roll the linear algebra to keep the
//! physcad kernel dependency-free):
//!
//! - Residuals are written once over a generic `Scalar` and evaluated with
//!   *dual numbers* to get exact analytic Jacobians (no finite differences,
//!   no second copy of each constraint for its derivative).
//! - LM step: solve (JᵀJ + λ·(1+diag(JᵀJ))) δ = -Jᵀr by Cholesky; the
//!   +λ floor keeps rank-deficient columns damped instead of blowing up.
//!   λ starts at 1e-3, ×0.1 on accepted steps (floor 1e-12), ×10 on
//!   rejected; bail-out at λ>1e12 => stalled/contradictory system, which
//!   is *reported*, never panicked.
//! - Rank / DOF: eigen-decomposition of JᵀJ via cyclic Jacobi; null-space
//!   eigenvectors show which *parameters* can still move. A greedy
//!   modified Gram-Schmidt pass over Jacobian rows names the *constraints*
//!   that are linearly dependent on earlier ones.

use super::{Constraint, Sketch};
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// Max scalar parameters one constraint term touches (angle: 2 lines = 8,
/// tangent line-arc: 4+4 = 8). Checked at assembly time.
const MAX_LOCAL: usize = 8;

#[derive(Clone, Debug)]
pub struct SolveOptions {
    pub max_iter: usize,
    /// Convergence gate on max |residual| (units: mm or rad per equation).
    pub tol: f64,
}

impl Default for SolveOptions {
    fn default() -> Self {
        Self {
            max_iter: 200,
            tol: 1e-12,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolveStatus {
    /// Every equation satisfied to within `tol`.
    Solved,
    /// Hit max_iter or LM damping blew up: residuals are *not* within tol.
    Unsolved,
}

#[derive(Clone, Debug)]
pub struct SolveReport {
    pub status: SolveStatus,
    pub iterations: usize,
    /// Infinity norm of the final residuals (mm / rad per equation).
    pub max_residual: f64,
    pub l2_residual: f64,
    pub n_params: usize,
    pub n_equations: usize,
    pub rank: usize,
    /// n_params - rank: rigid-body or leftover internal freedom.
    pub dof: usize,
    /// Singular values of J at the solution, descending.
    pub singular_values: Vec<f64>,
    /// Entity-owned parameter labels lying in the null space.
    pub free_params: Vec<String>,
    /// Constraint names linearly dependent on earlier constraints.
    pub redundant: Vec<String>,
    pub message: String,
}

impl std::fmt::Display for SolveReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}: {} eqs, {} params, rank {}, dof {}, max|res| {:.3e}, {} iters",
            match self.status {
                SolveStatus::Solved => "solved",
                SolveStatus::Unsolved => "UNSOLVED",
            },
            self.n_equations,
            self.n_params,
            self.rank,
            self.dof,
            self.max_residual,
            self.iterations,
        )?;
        if !self.redundant.is_empty() {
            write!(f, ", redundant: [{}]", self.redundant.join(", "))?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------- scalars

pub trait Scalar:
    Copy
    + Add<Output = Self>
    + AddAssign
    + Sub<Output = Self>
    + SubAssign
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
    + From<f64>
{
    fn sqrt_v(self) -> Self;
    fn atan2_v(y: Self, x: Self) -> Self;
}

impl Scalar for f64 {
    fn sqrt_v(self) -> Self {
        f64::sqrt(self)
    }
    fn atan2_v(y: Self, x: Self) -> Self {
        y.atan2(x)
    }
}

/// Dual number with `MAX_LOCAL` derivative seeds.
#[derive(Clone, Copy, Debug)]
struct Dual {
    v: f64,
    d: [f64; MAX_LOCAL],
}

impl Dual {
    fn var(v: f64, i: usize) -> Self {
        let mut d = [0.0; MAX_LOCAL];
        d[i] = 1.0;
        Self { v, d }
    }
}

impl Scalar for Dual {
    fn sqrt_v(self) -> Self {
        let r = self.v.sqrt();
        Self {
            v: r,
            // guard: sqrt is not differentiable at 0; degenerate geometry
            // (zero-length edge) yields 0 derivatives, the rank report covers it.
            d: if r > 0.0 {
                std::array::from_fn(|k| self.d[k] / (2.0 * r))
            } else {
                [0.0; MAX_LOCAL]
            },
        }
    }
    fn atan2_v(y: Self, x: Self) -> Self {
        let den = x.v * x.v + y.v * y.v;
        Self {
            v: y.v.atan2(x.v),
            d: if den > 0.0 {
                std::array::from_fn(|k| (x.v * y.d[k] - y.v * x.d[k]) / den)
            } else {
                [0.0; MAX_LOCAL]
            },
        }
    }
}

impl From<f64> for Dual {
    fn from(v: f64) -> Self {
        Self {
            v,
            d: [0.0; MAX_LOCAL],
        }
    }
}
impl Add for Dual {
    type Output = Self;
    fn add(mut self, o: Self) -> Self {
        self.v += o.v;
        for k in 0..MAX_LOCAL {
            self.d[k] += o.d[k];
        }
        self
    }
}
impl AddAssign for Dual {
    fn add_assign(&mut self, o: Self) {
        *self = *self + o;
    }
}
impl Sub for Dual {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        self + (-o)
    }
}
impl SubAssign for Dual {
    fn sub_assign(&mut self, o: Self) {
        *self = *self - o;
    }
}
impl Neg for Dual {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            v: -self.v,
            d: self.d.map(|x| -x),
        }
    }
}
#[allow(clippy::suspicious_arithmetic_impl)] // dual product rule legitimately mixes + and *
impl Mul for Dual {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self {
            v: self.v * o.v,
            d: std::array::from_fn(|k| self.d[k] * o.v + self.v * o.d[k]),
        }
    }
}
#[allow(clippy::suspicious_arithmetic_impl)] // dual quotient rule mixes -, * and /
impl Div for Dual {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        Self {
            v: self.v / o.v,
            d: std::array::from_fn(|k| (self.d[k] * o.v - self.v * o.d[k]) / (o.v * o.v)),
        }
    }
}

/// Normalize an angle target to (-pi, pi].
pub fn wrap_pi(mut a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    a = a.rem_euclid(tau);
    if a > std::f64::consts::PI {
        a -= tau;
    }
    a
}

// ------------------------------------------------------------ term residuals

/// Hypot written generic over Scalar (the 1e-300 floor only biases f64
/// values by <1e-150, keeping sqrt differentiable in Dual at degenerate
/// zero-length edges instead of emitting NaN).
fn hypot<S: Scalar>(dx: S, dy: S) -> S {
    (dx * dx + dy * dy + S::from(1e-300)).sqrt_v()
}

/// Signed distance from (px,py) to line (ax,ay)->(bx,by):
/// cross(dir, p-a)/|dir|; +side = rot90CCW of the direction.
fn side_dist<S: Scalar>(ax: S, ay: S, bx: S, by: S, px: S, py: S) -> S {
    let dx = bx - ax;
    let dy = by - ay;
    (dx * (py - ay) - dy * (px - ax)) / hypot(dx, dy)
}

/// Evaluate one constraint's residuals over slots laid out exactly as
/// `Sketch::term_params` produces. One implementation serves both the
/// f64 pass (values) and the dual pass (exact Jacobian rows).
fn eval_term<S: Scalar>(c: &Constraint, s: &[S]) -> Vec<S> {
    match c {
        Constraint::Coincident { .. } => vec![s[0] - s[2], s[1] - s[3]],
        Constraint::Distance { value, .. } => {
            vec![hypot(s[0] - s[2], s[1] - s[3]) - S::from(*value)]
        }
        Constraint::Angle { radians, .. } => {
            // slots: line a (4), line b (4); theta = CCW from dir(a) to dir(b)
            let (d1x, d1y) = (s[2] - s[0], s[3] - s[1]);
            let (d2x, d2y) = (s[6] - s[4], s[7] - s[5]);
            let cross = d1x * d2y - d1y * d2x;
            let dot = d1x * d2x + d1y * d2y;
            vec![S::atan2_v(cross, dot) - S::from(wrap_pi(*radians))]
        }
        Constraint::Horizontal { .. } => vec![s[3] - s[1]],
        Constraint::Vertical { .. } => vec![s[2] - s[0]],
        Constraint::Radius { value, .. } => vec![s[0] - S::from(*value)],
        Constraint::TangentLineCircle { .. } => {
            // slots: line (4), circle center (2), circle radius (1).
            // Squared form: sign-agnostic tangency (either side of the
            // line), polynomial-smooth, no |x| kink.
            let sd = side_dist(s[0], s[1], s[2], s[3], s[4], s[5]);
            vec![sd * sd - s[6] * s[6]]
        }
        Constraint::TangentLineArc { .. } => {
            // slots: line (4), arc center (2), arc start (2); r² = |s-c|²
            let sd = side_dist(s[0], s[1], s[2], s[3], s[4], s[5]);
            let (rx, ry) = (s[6] - s[4], s[7] - s[5]);
            vec![sd * sd - (rx * rx + ry * ry)]
        }
        Constraint::TangentCircles { .. } => {
            // slots: c1 (2) r1, c2 (2) r2
            vec![hypot(s[3] - s[0], s[4] - s[1]) - s[2] - s[5]]
        }
        Constraint::Fix { x, y, .. } => vec![s[0] - S::from(*x), s[1] - S::from(*y)],
    }
}

fn term_slots(sk: &Sketch, nc: &Constraint) -> Vec<usize> {
    let slots = sk.term_params(nc);
    assert!(
        slots.len() <= MAX_LOCAL,
        "constraint touches {} params > MAX_LOCAL {MAX_LOCAL}",
        slots.len()
    );
    slots
}

/// Residual vector of the whole system at parameter point x.
pub fn residuals(sk: &Sketch, x: &[f64]) -> Vec<f64> {
    let mut out = Vec::new();
    for nc in &sk.constraints {
        let slots = term_slots(sk, &nc.kind);
        let s: Vec<f64> = slots.iter().map(|&i| x[i]).collect();
        out.extend(eval_term(&nc.kind, &s));
    }
    out
}

/// Dense Jacobian (m rows × n cols) + residuals, one dual pass per term.
fn jacobian(sk: &Sketch, x: &[f64]) -> (Vec<Vec<f64>>, Vec<f64>) {
    let n = sk.n_params();
    let mut jac = Vec::new();
    let mut res = Vec::new();
    for nc in &sk.constraints {
        let slots = term_slots(sk, &nc.kind);
        let s: Vec<Dual> = slots
            .iter()
            .enumerate()
            .map(|(k, &p)| Dual::var(x[p], k))
            .collect();
        for r in eval_term(&nc.kind, &s) {
            let mut row = vec![0.0; n];
            for (k, &p) in slots.iter().enumerate() {
                // a term may read the same param twice; merge partials
                row[p] += r.d[k];
            }
            jac.push(row);
            res.push(r.v);
        }
    }
    (jac, res)
}

// ------------------------------------------------------------ linear algebra

/// Cholesky solve A δ = b for symmetric A; None if not positive definite.
// Row/col index arithmetic reads clearer than iterator gymnastics in
// numerical kernels (mission rule: `allow` with justification).
#[allow(clippy::needless_range_loop)]
fn cholesky_solve(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    let mut l = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = a[i][j];
            for k in 0..j {
                s -= l[i][k] * l[j][k];
            }
            if i == j {
                if s <= 0.0 {
                    return None;
                }
                l[i][i] = s.sqrt();
            } else {
                l[i][j] = s / l[j][j];
            }
        }
    }
    // forward substitution L y = b
    let mut y = vec![0.0; n];
    for i in 0..n {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i][k] * y[k];
        }
        y[i] = s / l[i][i];
    }
    // back substitution Lᵀ δ = y
    let mut d = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = y[i];
        for k in i + 1..n {
            s -= l[k][i] * d[k];
        }
        d[i] = s / l[i][i];
    }
    Some(d)
}

/// Cyclic Jacobi eigen-decomposition of a symmetric n×n matrix.
/// Returns (eigenvalues, eigenvectors-as-columns), sorted descending.
#[allow(clippy::needless_range_loop)] // column-stride rotations: indexed loops are the clear form
fn jacobi_eigen(a: &mut [Vec<f64>], n: usize) -> (Vec<f64>, Vec<Vec<f64>>) {
    let mut v = vec![vec![0.0; n]; n];
    for i in 0..n {
        v[i][i] = 1.0;
    }
    for _sweep in 0..100 {
        let mut off = 0.0;
        for i in 0..n {
            for j in i + 1..n {
                off += a[i][j] * a[i][j];
            }
        }
        if off <= 1e-30 {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                let apq = a[p][q];
                if apq.abs() <= 1e-18 {
                    continue;
                }
                let app = a[p][p];
                let aqq = a[q][q];
                let tau = (aqq - app) / (2.0 * apq);
                let t = tau.signum() / (tau.abs() + (tau * tau + 1.0).sqrt());
                let t = if tau == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let akp = a[k][p];
                    let akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let akp = a[p][k];
                    let akq = a[q][k];
                    a[p][k] = c * akp - s * akq;
                    a[q][k] = s * akp + c * akq;
                }
                for k in 0..n {
                    let vkp = v[k][p];
                    let vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    let mut evals: Vec<(f64, usize)> = (0..n).map(|i| (a[i][i], i)).collect();
    evals.sort_by(|x, y| y.0.total_cmp(&x.0));
    let vals = evals.iter().map(|(e, _)| *e).collect();
    let vecs = (0..n)
        .map(|col| (0..n).map(|row| v[row][col]).collect())
        .collect();
    (vals, vecs)
}

/// Human labels for each global param: "a.x", "a.y", "c.r" …
fn param_labels(sk: &Sketch) -> Vec<String> {
    let mut out = Vec::new();
    for (name, e) in &sk.entities {
        match e {
            super::Entity::Point { .. } => {
                out.push(format!("{name}.x"));
                out.push(format!("{name}.y"));
            }
            super::Entity::Circle { .. } => out.push(format!("{name}.r")),
            _ => {}
        }
    }
    out
}

/// Indices of constraint rows linearly dependent on earlier rows (greedy
/// modified Gram-Schmidt over the Jacobian, deterministic order).
fn redundant_rows(jac: &[Vec<f64>]) -> Vec<usize> {
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut red = Vec::new();
    for (i, row) in jac.iter().enumerate() {
        let mut v = row.clone();
        for q in &basis {
            let d: f64 = v.iter().zip(q).map(|(a, b)| a * b).sum();
            for (a, b) in v.iter_mut().zip(q) {
                *a -= d * b;
            }
        }
        let norm: f64 = v.iter().map(|c| c * c).sum::<f64>().sqrt();
        let row_norm: f64 = row.iter().map(|c| c * c).sum::<f64>().sqrt();
        if norm <= 1e-10 * row_norm.max(1.0) {
            red.push(i);
        } else {
            basis.push(v.iter().map(|c| c / norm).collect());
        }
    }
    red
}

// -------------------------------------------------------------------- LM

/// Solve the sketch. `x` is updated in place (DOF-layout order); the report
/// describes residuals, rank, DOF and redundancy at the returned point.
pub fn solve_system(sk: &Sketch, x: &mut [f64], opts: &SolveOptions) -> SolveReport {
    let n = x.len();
    let mut iterations = 0usize;
    let mut lambda = 1e-3f64;
    let mut message = String::new();

    let mut r = residuals(sk, x);
    let norm = |r: &[f64]| -> f64 { r.iter().map(|v| v * v).sum::<f64>().sqrt() };
    let maxabs = |r: &[f64]| -> f64 { r.iter().fold(0.0f64, |a, v| a.max(v.abs())) };

    let mut stalled = false;
    for it in 0..opts.max_iter {
        iterations = it + 1;
        if maxabs(&r) <= opts.tol {
            break;
        }
        let (jac, _) = jacobian(sk, x);
        // normal equations
        let mut jtj = vec![vec![0.0; n]; n];
        let mut grad = vec![0.0; n];
        for (row, rv) in jac.iter().zip(&r) {
            for i in 0..n {
                let vi = row[i];
                if vi == 0.0 {
                    continue;
                }
                grad[i] += vi * rv;
                for j in 0..n {
                    jtj[i][j] += vi * row[j];
                }
            }
        }
        let diag: Vec<f64> = (0..n).map(|i| jtj[i][i]).collect();
        loop {
            let mut a = jtj.clone();
            for i in 0..n {
                a[i][i] += lambda * (1.0 + diag[i]);
            }
            let b: Vec<f64> = grad.iter().map(|g| -g).collect();
            if let Some(step) = cholesky_solve(&a, &b) {
                let xt: Vec<f64> = x.iter().zip(&step).map(|(a, s)| a + s).collect();
                let rt = residuals(sk, &xt);
                if norm(&rt) < norm(&r) {
                    r = rt;
                    x.copy_from_slice(&xt);
                    lambda = (lambda * 0.1).max(1e-12);
                    break;
                }
            }
            lambda *= 10.0;
            if lambda > 1e12 {
                stalled = true;
                break;
            }
        }
        if stalled {
            message = format!(
                "LM damping blew up (λ>1e12): constraints are contradictory or the initial guess is too far; max|res| = {:.3e}",
                maxabs(&r)
            );
            break;
        }
    }

    // final diagnostics at the returned point
    let (jac, _) = jacobian(sk, x);
    let m = jac.len();
    let mut jtj = vec![vec![0.0; n]; n];
    for row in &jac {
        for i in 0..n {
            let vi = row[i];
            if vi == 0.0 {
                continue;
            }
            for j in 0..n {
                jtj[i][j] += vi * row[j];
            }
        }
    }
    let (mut eigvals, eigvecs) = if n > 0 {
        jacobi_eigen(&mut jtj, n)
    } else {
        (Vec::new(), Vec::new())
    };
    for e in eigvals.iter_mut() {
        if *e < 0.0 {
            *e = 0.0; // symmetric round-off floor
        }
    }
    let emax = eigvals.iter().fold(0.0f64, |a, e| a.max(*e));
    let etol = 1e-10 * emax.max(1.0);
    let rank = eigvals.iter().filter(|e| **e > etol).count();
    let labels = param_labels(sk);
    let mut free_params = Vec::new();
    for (k, e) in eigvals.iter().enumerate() {
        if *e <= etol {
            for i in 0..n {
                if eigvecs[k][i].abs() > 1e-6 && !free_params.contains(&labels[i]) {
                    free_params.push(labels[i].clone());
                }
            }
        }
    }
    free_params.sort();
    let red_idx = redundant_rows(&jac);
    let redundant: Vec<String> = red_idx
        .iter()
        .map(|row_i| {
            // map row -> constraint: rebuild cumulative row ends
            let mut acc = 0usize;
            for nc in &sk.constraints {
                acc += Sketch::term_equations(&nc.kind);
                if *row_i < acc {
                    return nc.name.clone();
                }
            }
            unreachable!("row index out of range")
        })
        .collect();

    let status = if maxabs(&r) <= opts.tol {
        SolveStatus::Solved
    } else {
        SolveStatus::Unsolved
    };
    if message.is_empty() {
        message = match status {
            SolveStatus::Solved if rank == n && red_idx.is_empty() => {
                "fully constrained".to_string()
            }
            SolveStatus::Solved => {
                let mut parts = Vec::new();
                if rank < n {
                    parts.push(format!("under-constrained: {} dof", n - rank));
                }
                if !red_idx.is_empty() {
                    parts.push(format!("{} redundant constraint(s)", red_idx.len()));
                }
                parts.join("; ")
            }
            SolveStatus::Unsolved => format!(
                "did not converge: max|res| {:.3e} after {iterations} iters",
                maxabs(&r)
            ),
        };
    }

    SolveReport {
        status,
        iterations,
        max_residual: maxabs(&r),
        l2_residual: norm(&r),
        n_params: n,
        n_equations: m,
        rank,
        dof: n - rank,
        singular_values: eigvals.iter().map(|e| e.sqrt()).collect(),
        free_params,
        redundant,
        message,
    }
}

/// Convenience: solve from the sketch's current coords into a solved copy.
pub fn solve(sk: &Sketch) -> (Sketch, SolveReport) {
    solve_with(sk, &SolveOptions::default())
}

pub fn solve_with(sk: &Sketch, opts: &SolveOptions) -> (Sketch, SolveReport) {
    let mut x = sk.params_vec();
    let report = solve_system(sk, &mut x, opts);
    (sk.with_solution(&x), report)
}
