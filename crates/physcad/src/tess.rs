//! f64 triangle tessellation of kernel primitives (CSG input side).
//!
//! `primitives.rs` meshes are f32 and good enough to *display*; the CSG
//! volume gates compare against analytic solids at 1e-6 absolute, which
//! needs f64 trig — f32 angle accumulation at n=8192 segments leaves a
//! ~1e-6 closing error in the polygon area (angle ulp ~7e-7 rad at 2π).
//!
//! Winding follows the M2 rule that caught hand-listed corners: derive
//! each face from a basis with u × v = n_out and walk (s,t) corners in
//! that basis, so CCW-from-outside holds by construction.

use glam::DVec3;

/// Triangle soup in f64 (kernel-precision geometry).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh64 {
    pub positions: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
}

impl Mesh64 {
    fn push_vert(&mut self, p: DVec3) -> u32 {
        let i = self.positions.len() as u32;
        self.positions.push(p.to_array());
        i
    }

    fn push_tri(&mut self, a: u32, b: u32, c: u32) {
        self.indices.extend_from_slice(&[a, b, c]);
    }

    fn push_quad(&mut self, a: u32, b: u32, c: u32, d: u32) {
        self.push_tri(a, b, c);
        self.push_tri(a, c, d);
    }

    /// Signed volume via the divergence theorem; positive for CCW-outside.
    pub fn signed_volume(&self) -> f64 {
        let mut sum = 0.0f64;
        for tri in self.indices.chunks(3) {
            let [a, b, c] = [
                DVec3::from(self.positions[tri[0] as usize]),
                DVec3::from(self.positions[tri[1] as usize]),
                DVec3::from(self.positions[tri[2] as usize]),
            ];
            sum += a.dot(b.cross(c)) / 6.0;
        }
        sum
    }
}

/// Axis-aligned box centered on the origin (x=w, y=h, z=d).
pub fn box64(w: f64, h: f64, d: f64) -> Mesh64 {
    assert!(
        w > 0.0 && h > 0.0 && d > 0.0,
        "box extents must be positive"
    );
    let half = DVec3::new(w, h, d) / 2.0;
    let (x, y, z) = (DVec3::X, DVec3::Y, DVec3::Z);
    let mut m = Mesh64::default();
    // Same (n, u, v) face table verified in primitives.rs M2 tests:
    // u.cross(v) == n, corner walk (s,t) CCW from outside.
    let faces: [(DVec3, DVec3, DVec3); 6] = [
        (x, y, z),  // +X
        (-x, z, y), // -X
        (y, z, x),  // +Y
        (-y, x, z), // -Y
        (z, x, y),  // +Z
        (-z, y, x), // -Z
    ];
    for (n, u, v) in faces {
        debug_assert!(u.cross(v) == n);
        let (hu, hv) = (half.dot(u.abs()), half.dot(v.abs()));
        let nh = half * n;
        let a = m.push_vert(nh - u * hu - v * hv);
        let b = m.push_vert(nh + u * hu - v * hv);
        let c = m.push_vert(nh + u * hu + v * hv);
        let d = m.push_vert(nh - u * hu + v * hv);
        m.push_quad(a, b, c, d);
    }
    m
}

/// Cylinder around +Y, centered on origin, `segs` inscribed sides.
pub fn cylinder64(radius: f64, height: f64, segs: usize) -> Mesh64 {
    assert!(
        radius > 0.0 && height > 0.0,
        "cylinder r/h must be positive"
    );
    assert!(segs >= 3, "cylinder needs >= 3 segments");
    let hy = height / 2.0;
    let step = std::f64::consts::TAU / segs as f64;
    // Precomputed rings; index math wraps modulo so seams share the exact
    // same f64 values (M2 lesson: identical args -> identical bits).
    let ring = |y: f64| -> Vec<DVec3> {
        (0..segs)
            .map(|i| {
                let a = step * i as f64;
                DVec3::new(radius * a.cos(), y, radius * a.sin())
            })
            .collect()
    };
    let top = ring(hy);
    let bot = ring(-hy);
    let mut m = Mesh64::default();
    let id: Vec<u32> = top
        .iter()
        .chain(bot.iter())
        .map(|p| m.push_vert(*p))
        .collect();
    let (t, b) = id.split_at(segs);

    // Sides: quad (bot[i], top[i], top[j], bot[j]) — verified in cylinder_mesh M2.
    for i in 0..segs {
        let j = (i + 1) % segs;
        m.push_quad(b[i], t[i], t[j], b[j]);
    }
    // Caps: fan from exact center vertex, same winding as M2.
    let tc = m.push_vert(DVec3::new(0.0, hy, 0.0));
    let bc = m.push_vert(DVec3::new(0.0, -hy, 0.0));
    for i in 0..segs {
        let j = (i + 1) % segs;
        m.push_tri(tc, t[j], t[i]);
        m.push_tri(bc, b[i], b[j]);
    }
    m
}

/// UV sphere centered on origin: `rings` latitude bands, `segs` longitudes.
/// All vertices share the position grid, so the mesh is welded by
/// construction (poles are single vertices; longitude seams index modulo).
pub fn sphere64(radius: f64, rings: usize, segs: usize) -> Mesh64 {
    assert!(radius > 0.0, "sphere radius must be positive");
    assert!(rings >= 2 && segs >= 3, "sphere needs rings>=2, segs>=3");
    let dphi = std::f64::consts::PI / rings as f64;
    let dtheta = std::f64::consts::TAU / segs as f64;
    let at = |i: usize, j: usize| -> DVec3 {
        // j = 0..=rings latitude rows north->south; wrap i modulo (segs).
        let phi = std::f64::consts::FRAC_PI_2 - j as f64 * dphi;
        let theta = (i % segs) as f64 * dtheta;
        DVec3::new(
            radius * phi.cos() * theta.cos(),
            radius * phi.sin(),
            radius * phi.cos() * theta.sin(),
        )
    };
    let mut m = Mesh64::default();
    let north = m.push_vert(at(0, 0));
    let mut rows: Vec<Vec<u32>> = Vec::with_capacity(rings - 1);
    for j in 1..rings {
        rows.push((0..segs).map(|i| m.push_vert(at(i, j))).collect());
    }
    let south = m.push_vert(at(0, rings));

    // Winding: basis (u, v) = (theta direction, south direction) satisfies
    // du x dv = outward (checked at equator in module tests); so quads walk
    // (i,j), (i+1,j), (i+1,j+1), (i,j+1). North cap: (pole, i, i+1);
    // south cap mirrors it.
    // North cap: degenerate quad (NP,NP,theta_j,theta_i) leaves the fan
    // triangle (NP, theta_{i+1}, theta_i); south cap mirrors via the other
    // degenerate quad corner: (theta_i, theta_{i+1}, SP).
    for i in 0..segs {
        let j = (i + 1) % segs;
        m.push_tri(north, rows[0][j], rows[0][i]);
    }
    for r in 0..rings - 2 {
        for i in 0..segs {
            let j = (i + 1) % segs;
            m.push_quad(rows[r][i], rows[r][j], rows[r + 1][j], rows[r + 1][i]);
        }
    }
    let last = rings - 2;
    for i in 0..segs {
        let j = (i + 1) % segs;
        m.push_tri(rows[last][i], rows[last][j], south);
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Every face of a centered convex primitive must have its normal
    /// (from CCW winding) pointing away from the origin.
    fn assert_outward_winding(m: &Mesh64) {
        for tri in m.indices.chunks(3) {
            let p = [
                DVec3::from(m.positions[tri[0] as usize]),
                DVec3::from(m.positions[tri[1] as usize]),
                DVec3::from(m.positions[tri[2] as usize]),
            ];
            let n = (p[1] - p[0]).cross(p[2] - p[0]);
            let c = (p[0] + p[1] + p[2]) / 3.0;
            assert!(n.dot(c) > 0.0, "face winding must be outward");
        }
    }

    #[test]
    fn box64_exact_volume_and_winding() {
        let m = box64(3.0, 4.0, 5.0);
        assert_eq!(m.signed_volume(), 60.0, "box volume must be exact in f64");
        assert_outward_winding(&m);
    }

    #[test]
    fn cylinder64_inscribed_polygon_volume_bound() {
        // Inscribed n-gon prism: V = (n/2) r^2 sin(2pi/n) h — exact f64
        // expectation; mesh must match it *and* approach pi r^2 h.
        let (r, h, n) = (1.0f64, 2.0, 8192usize);
        let m = cylinder64(r, h, n);
        assert_outward_winding(&m);
        let inscribed = 0.5 * n as f64 * r * r * (std::f64::consts::TAU / n as f64).sin() * h;
        let err_inscr = (m.signed_volume() - inscribed).abs();
        let err_analytic = (m.signed_volume() - PI * r * r * h).abs();
        eprintln!("cyl64 n={n}: vs inscribed {err_inscr:.3e}, vs analytic {err_analytic:.3e}");
        assert!(
            err_inscr < 1e-10,
            "mesh != inscribed polygon: {err_inscr:e}"
        );
        assert!(
            err_analytic < 1e-6,
            "cylinder64 not within 1e-6 of analytic"
        );
    }

    #[test]
    fn sphere64_winding_and_volume() {
        let m = sphere64(0.25, 1024, 1024);
        assert_outward_winding(&m);
        let (r, v) = (0.25f64, m.signed_volume());
        let analytic = 4.0 / 3.0 * PI * r.powi(3);
        let err = (v - analytic).abs();
        eprintln!("sph64 r={r}: mesh {v:.9}, analytic {analytic:.9}, err {err:.3e}");
        assert!(v < analytic, "UV sphere is inscribed, must under-estimate");
        assert!(err < 1e-6, "sphere64 not within 1e-6 of analytic: {err:e}");
    }

    #[test]
    fn small_tessellations_have_expected_counts() {
        let m = cylinder64(1.0, 1.0, 8);
        assert_eq!(m.indices.len() / 3, 4 * 8); // sides 2n + caps 2n
        let m = sphere64(1.0, 4, 6);
        // 2n pole tris + (rings-2)*2n quad tris
        assert_eq!(m.indices.len() / 3, 2 * 6 + (4 - 2) * 2 * 6);
    }
}
