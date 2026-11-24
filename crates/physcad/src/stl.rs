//! Binary STL export/import.
//!
//! Deterministic bytes: the 80-byte header is a fixed string (no
//! timestamps), attribute counters are 0, normals are recomputed from
//! winding in f32. Same mesh in -> same bytes out, which is what the
//! "bit-stable rebuild" gate checks.

use crate::primitives::Mesh;

const fn padded_header(text: &[u8]) -> [u8; 80] {
    let mut out = [b' '; 80];
    let mut i = 0;
    while i < text.len() && i < 80 {
        out[i] = text[i];
        i += 1;
    }
    out
}

static HEADER: [u8; 80] = padded_header(b"physcad M3 stl - deterministic (units mm)");

#[derive(Clone, Debug)]
pub struct StlError(pub String);

impl std::fmt::Display for StlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for StlError {}

/// Write a triangle mesh as binary STL. The mesh's own normals are
/// ignored: STL carries per-face normals, and deriving them from winding
/// keeps the bytes a pure function of geometry.
pub fn to_stl_binary(mesh: &Mesh) -> Vec<u8> {
    let n_tri = mesh.triangle_count();
    let mut out = Vec::with_capacity(84 + 50 * n_tri);
    out.extend_from_slice(&HEADER);
    out.extend_from_slice(&(n_tri as u32).to_le_bytes());
    for tri in mesh.indices.chunks(3) {
        let [a, b, c] = [
            glam::Vec3::from(mesh.positions[tri[0] as usize]),
            glam::Vec3::from(mesh.positions[tri[1] as usize]),
            glam::Vec3::from(mesh.positions[tri[2] as usize]),
        ];
        let n = (b - a).cross(c - a);
        let n = if n.length_squared() > 0.0 {
            n.normalize()
        } else {
            glam::Vec3::ZERO
        };
        for v in [n, a, b, c] {
            for f in v.to_array() {
                out.extend_from_slice(&f.to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

/// Read binary STL back into a triangle-soup Mesh (flat normals). ASCII
/// STL is not implemented — binary is the interchange we control.
pub fn from_stl_binary(bytes: &[u8]) -> Result<Mesh, StlError> {
    if bytes.len() < 84 {
        return Err(StlError(format!("truncated STL: {} bytes", bytes.len())));
    }
    let n_tri = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    let want = 84 + 50 * n_tri;
    if bytes.len() < want {
        return Err(StlError(format!(
            "STL declares {n_tri} triangles (needs {want} bytes), got {}",
            bytes.len()
        )));
    }
    let mut mesh = Mesh::new();
    for rec in bytes[84..want].as_chunks::<50>().0 {
        let f = |k: usize| -> [f32; 3] {
            let raw: [u8; 12] = rec[k * 12..k * 12 + 12].try_into().unwrap();
            [
                f32::from_le_bytes(raw[0..4].try_into().unwrap()),
                f32::from_le_bytes(raw[4..8].try_into().unwrap()),
                f32::from_le_bytes(raw[8..12].try_into().unwrap()),
            ]
        };
        let (n, a, b, c) = (f(0), f(1), f(2), f(3));
        // Keep the declared normal as-is: bytes round-trip must be stable
        // against our own writer, which derives normals from winding.
        for p in [a, b, c] {
            let i = mesh.positions.len() as u32;
            mesh.positions.push(p);
            mesh.normals.push(n);
            mesh.indices.push(i);
        }
    }
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::box_mesh;

    #[test]
    fn stl_layout_and_size_exact() {
        let m = box_mesh(1.0, 2.0, 3.0); // 12 tris
        let b = to_stl_binary(&m);
        assert_eq!(b.len(), 84 + 50 * 12);
        assert_eq!(&b[0..80], &HEADER[..]);
        assert_eq!(u32::from_le_bytes(b[80..84].try_into().unwrap()), 12);
        // deterministic: same input -> same bytes
        assert_eq!(b, to_stl_binary(&m));
    }

    #[test]
    fn stl_round_trip_preserves_volume_and_manifold() {
        let m = box_mesh(2.0, 3.0, 4.0);
        let back = from_stl_binary(&to_stl_binary(&m)).unwrap();
        assert_eq!(back.triangle_count(), 12);
        assert!((back.signed_volume() - 24.0).abs() < 1e-4);
        back.closed_manifold_check().unwrap();
    }

    #[test]
    fn truncated_stl_rejected() {
        let m = box_mesh(1.0, 1.0, 1.0);
        let mut b = to_stl_binary(&m);
        b.truncate(b.len() - 25);
        assert!(from_stl_binary(&b).is_err());
        assert!(from_stl_binary(b"[garbage").is_err());
    }
}
