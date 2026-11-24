//! Triangle meshes and parametric primitive generators.
//!
//! Convention: Y-up world, primitives centered on their own origin.
//! Every generator bakes per-vertex normals (flat normals on the box,
//! smooth on cylinder sides/caps and torus).

use glam::Vec3;

/// Triangle soup with parallel `positions`/`normals` and CCW `indices`
/// (CCW when viewed from outside the solid).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

impl Mesh {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// Push one vertex, returning its index.
    fn push_vert(&mut self, p: Vec3, n: Vec3) -> u32 {
        let i = self.positions.len() as u32;
        self.positions.push(p.to_array());
        self.normals.push(n.normalize().to_array());
        i
    }

    fn push_tri(&mut self, a: u32, b: u32, c: u32) {
        self.indices.extend_from_slice(&[a, b, c]);
    }

    fn push_quad(&mut self, a: u32, b: u32, c: u32, d: u32) {
        self.push_tri(a, b, c);
        self.push_tri(a, c, d);
    }

    /// Signed volume via the divergence theorem over the triangle surface.
    /// Only meaningful for closed, consistently-oriented meshes.
    pub fn signed_volume(&self) -> f32 {
        let mut sum = 0.0f32;
        for tri in self.indices.chunks(3) {
            let [a, b, c] = [
                Vec3::from(self.positions[tri[0] as usize]),
                Vec3::from(self.positions[tri[1] as usize]),
                Vec3::from(self.positions[tri[2] as usize]),
            ];
            sum += a.dot(b.cross(c)) / 6.0;
        }
        sum
    }

    /// Check the closed-manifold invariant on the *position* deduplicated
    /// mesh: every directed edge appears exactly once, and its reverse
    /// appears exactly once. Returns (deduped V, E, F) on success.
    pub fn closed_manifold_check(&self) -> Result<(usize, usize, usize), String> {
        if !self.indices.len().is_multiple_of(3) {
            return Err(format!(
                "index count {} not a multiple of 3",
                self.indices.len()
            ));
        }
        if self.positions.len() != self.normals.len() {
            return Err("positions/normals length mismatch".into());
        }

        // Deduplicate vertices by exact position bits (primitives use exact
        // shared coordinates, so bit-equality is safe here).
        let mut dedup: std::collections::HashMap<[u32; 3], u32> = std::collections::HashMap::new();
        let mut remap = vec![u32::MAX; self.positions.len()];
        for (i, p) in self.positions.iter().enumerate() {
            let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
            let next = dedup.len() as u32;
            let id = *dedup.entry(key).or_insert(next);
            remap[i] = id;
        }
        let v = dedup.len();

        let mut edges: std::collections::HashMap<(u32, u32), u32> =
            std::collections::HashMap::new();
        for tri in self.indices.chunks(3) {
            let t = [
                remap[tri[0] as usize],
                remap[tri[1] as usize],
                remap[tri[2] as usize],
            ];
            if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] {
                return Err(format!("degenerate triangle: {t:?}"));
            }
            for [a, b] in [[t[0], t[1]], [t[1], t[2]], [t[2], t[0]]] {
                *edges.entry((a, b)).or_insert(0) += 1;
            }
        }
        let f = self.triangle_count();
        let e = edges.len() / 2;
        for (&(a, b), &count) in &edges {
            if count != 1 {
                return Err(format!("directed edge ({a},{b}) used {count} times"));
            }
            if !edges.contains_key(&(b, a)) {
                return Err(format!(
                    "edge ({a},{b}) has no opposite ({b},{a}) — open or bad winding"
                ));
            }
        }
        Ok((v, e, f))
    }
}

/// Axis-aligned box of the given width (x), height (y), depth (z).
pub fn box_mesh(width: f32, height: f32, depth: f32) -> Mesh {
    assert!(
        width > 0.0 && height > 0.0 && depth > 0.0,
        "box extents must be positive"
    );
    let half = Vec3::new(width, height, depth) / 2.0;
    let (x, y, z) = (Vec3::X, Vec3::Y, Vec3::Z);
    let mut m = Mesh::new();
    // Faces as (true outward normal n, in-plane basis u, v) with
    // u.cross(v) == n. Walking corners (s,t) = (-,-),(+,-),(+,+),(-,+)
    // in that basis is then CCW as seen from outside, by construction.
    let faces: [(Vec3, Vec3, Vec3); 6] = [
        (x, y, z),  // +X
        (-x, z, y), // -X
        (y, z, x),  // +Y
        (-y, x, z), // -Y
        (z, x, y),  // +Z
        (-z, y, x), // -Z
    ];
    for (n, u, v) in faces {
        debug_assert!(u.cross(v) == n, "face basis must satisfy u x v = n");
        let (hu, hv) = (half.dot(u.abs()), half.dot(v.abs()));
        let nh = half * n;
        let a = m.push_vert(nh - u * hu - v * hv, n);
        let b = m.push_vert(nh + u * hu - v * hv, n);
        let c = m.push_vert(nh + u * hu + v * hv, n);
        let d = m.push_vert(nh - u * hu + v * hv, n);
        m.push_quad(a, b, c, d);
    }
    m
}

/// Cylinder around +Y: `segments` sides, flat caps, smooth side normals.
pub fn cylinder_mesh(radius: f32, height: f32, segments: u32) -> Mesh {
    assert!(
        radius > 0.0 && height > 0.0,
        "cylinder radius/height must be positive"
    );
    assert!(segments >= 3, "cylinder needs at least 3 segments");
    let hy = height / 2.0;
    let step = std::f32::consts::TAU / segments as f32;
    let ring = |y: f32, i: u32| -> Vec3 {
        let a = step * i as f32;
        Vec3::new(radius * a.cos(), y, radius * a.sin())
    };
    let mut m = Mesh::new();

    // Side quads (smooth normals = radial direction). Corner walk in the
    // (Y, tangent) basis: Y x tangent = outward radius, so CCW-from-outside.
    for i in 0..segments {
        let j = (i + 1) % segments;
        let a0 = m.push_vert(ring(-hy, i), ring(-hy, i).with_y(0.0));
        let b0 = m.push_vert(ring(hy, i), ring(hy, i).with_y(0.0));
        let c0 = m.push_vert(ring(hy, j), ring(hy, j).with_y(0.0));
        let d0 = m.push_vert(ring(-hy, j), ring(-hy, j).with_y(0.0));
        m.push_quad(a0, b0, c0, d0);
    }

    // Caps: fan from center vertex. Top sees CCW as (center, ring(j), ring(i));
    // bottom mirrors it.
    let top_c = m.push_vert(Vec3::new(0.0, hy, 0.0), Vec3::Y);
    let bot_c = m.push_vert(Vec3::new(0.0, -hy, 0.0), Vec3::NEG_Y);
    for i in 0..segments {
        let j = (i + 1) % segments;
        let a = m.push_vert(ring(hy, j), ring(hy, j).with_y(0.0));
        let b = m.push_vert(ring(hy, i), ring(hy, i).with_y(0.0));
        m.push_tri(top_c, a, b);
        let a = m.push_vert(ring(-hy, i), ring(-hy, i).with_y(0.0));
        let b = m.push_vert(ring(-hy, j), ring(-hy, j).with_y(0.0));
        m.push_tri(bot_c, a, b);
    }
    m
}

/// Torus around the Y axis: major circle radius `major_radius` in the XZ
/// plane, tube radius `minor_radius`, smooth normals.
pub fn torus_mesh(
    major_radius: f32,
    minor_radius: f32,
    major_segments: u32,
    minor_segments: u32,
) -> Mesh {
    assert!(
        major_radius > minor_radius && minor_radius > 0.0,
        "need major_radius > minor_radius > 0"
    );
    assert!(
        major_segments >= 3 && minor_segments >= 3,
        "torus needs at least 3 segments each"
    );
    let du = std::f32::consts::TAU / major_segments as f32;
    let dv = std::f32::consts::TAU / minor_segments as f32;
    let at = |u: u32, v: u32| -> (Vec3, Vec3) {
        // Wrap first: seam vertices then reuse the *identical* trig args as
        // the leading row/column, so position dedup merges them bitwise.
        let (a, b) = (
            du * (u % major_segments) as f32,
            dv * (v % minor_segments) as f32,
        );
        (
            Vec3::new(
                (major_radius + minor_radius * b.cos()) * a.cos(),
                minor_radius * b.sin(),
                (major_radius + minor_radius * b.cos()) * a.sin(),
            ),
            Vec3::new(b.cos() * a.cos(), b.sin(), b.cos() * a.sin()),
        )
    };
    let mut m = Mesh::new();
    for i in 0..major_segments {
        for j in 0..minor_segments {
            let (p00, n00) = at(i, j);
            let (p01, n01) = at(i, j + 1);
            let (p11, n11) = at(i + 1, j + 1);
            let (p10, n10) = at(i + 1, j);
            let a = m.push_vert(p00, n00);
            let b = m.push_vert(p01, n01);
            let c = m.push_vert(p11, n11);
            let d = m.push_vert(p10, n10);
            m.push_quad(a, b, c, d);
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn euler(m: &Mesh) -> i64 {
        let (v, e, f) = m.closed_manifold_check().expect("closed manifold");
        v as i64 - e as i64 + f as i64
    }

    #[test]
    fn box_is_closed_manifold_sphere_topology() {
        let m = box_mesh(2.0, 3.0, 4.0);
        assert_eq!(m.triangle_count(), 12);
        let (v, e, f) = m.closed_manifold_check().unwrap();
        assert_eq!((v, e, f), (8, 18, 12));
        assert_eq!(euler(&m), 2, "box must be topologically a sphere");
    }

    #[test]
    fn cylinder_is_closed_manifold_sphere_topology() {
        let m = cylinder_mesh(1.0, 2.0, 24);
        let (v, e, f) = m.closed_manifold_check().unwrap();
        // V = 2 centers + 2 rings of 24 positions; F = 48 side + 48 cap tris
        assert_eq!((v, e, f), (50, 144, 96));
        assert_eq!(euler(&m), 2);
    }

    #[test]
    fn torus_is_closed_manifold_torus_topology() {
        let m = torus_mesh(2.0, 0.5, 32, 16);
        let (v, e, f) = m.closed_manifold_check().unwrap();
        assert_eq!((v, e, f), (512, 1536, 1024));
        assert_eq!(euler(&m), 0, "torus Euler characteristic must be 0");
    }

    #[test]
    fn box_volume_matches_analytic_exactly() {
        let m = box_mesh(2.0, 3.0, 4.0);
        assert!((m.signed_volume() - 24.0).abs() < 1e-4);
    }

    #[test]
    fn cylinder_volume_within_discretization_error() {
        let (r, h) = (1.0f32, 2.0f32);
        let m = cylinder_mesh(r, h, 64);
        let analytic = PI * r * r * h;
        let rel = (m.signed_volume() - analytic).abs() / analytic;
        assert!(rel < 5e-3, "relative volume error {rel:.2e} too large");
    }

    #[test]
    fn torus_volume_within_discretization_error() {
        let (rr, r) = (2.0f32, 0.5f32);
        let m = torus_mesh(rr, r, 96, 48);
        let analytic = 2.0 * PI * PI * rr * r * r;
        let rel = (m.signed_volume() - analytic).abs() / analytic;
        assert!(rel < 5e-3, "relative volume error {rel:.2e} too large");
    }

    #[test]
    fn open_mesh_is_rejected() {
        let mut m = box_mesh(1.0, 1.0, 1.0);
        // Drop one triangle: the mesh must fail the manifold check.
        m.indices.truncate(m.indices.len() - 3);
        assert!(m.closed_manifold_check().is_err());
    }
}
