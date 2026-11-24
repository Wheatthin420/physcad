//! PhysCAD core: mesh data + parametric primitives + orbit camera.
//! Deliberately GPU-free and UI-free so geometry can be unit-tested headless.

pub mod camera;
pub mod primitives;

pub use camera::OrbitCamera;
pub use primitives::Mesh;
