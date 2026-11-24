//! PhysCAD core: mesh data + parametric primitives + orbit camera +
//! f64 CSG kernel (manifold3d) + model-file/STL I/O.
//! Deliberately GPU-free and UI-free so geometry can be unit-tested headless.

pub mod camera;
pub mod csg;
pub mod expr;
pub mod model;
pub mod primitives;
pub mod sketch;
pub mod stl;
pub mod sweep;
pub mod tess;

pub use camera::OrbitCamera;
pub use csg::{GeoGraph, GeoNode, Solid};
pub use model::Model;
pub use primitives::Mesh;
pub use sketch::Sketch;
pub use sketch::loops::Region2;
pub use sweep::{Axis2, SweepError, extrude_region, revolve_region, triangulate_region};
