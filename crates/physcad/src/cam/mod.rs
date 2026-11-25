//! M7: CAM draft — adaptive (offset-ring) 2D pocketing, GRBL-dialect
//! G-code emission, and a cut simulator cross-checked against kernel CSG.
//!
//! Mission-ladder text: "adaptive 2D pocket over a heightfield of the
//! mesh -> G-code text for GRBL dialect + a software stock/cut
//! simulator". Concretely this module ships:
//!
//! - `plan`: raster the pocket region (sketch `Region2`, islands
//!   included), Euclidean distance transform, contour-parallel offset
//!   rings at level `r + k*(2r - stepover)` (plus a core level when the
//!   medial axis is deeper than one ring radius), marching-squares
//!   tracing, Ramer-Douglas-Peucker simplification, bridge linking with
//!   an explicit wall-clearance check, and a coverage self-certification
//!   (every region pixel is within `r + 2*pixel` of some cut path).
//!   Trochoidal high-efficiency roughing is deferred; see docs/CAM.md.
//! - `gcode`: typed `Motion` IR -> GRBL-dialect text (emitter) plus our
//!   own strict grammar validator (the dialect gate). The emitter is
//!   never its own judge: the `gcode` crate (third-party, MIT OR
//!   Apache-2.0) parses every emitted program in tests, and `sim`
//!   replays from that parser's output.
//! - `sim`: G-code text -> modal state -> swept flat-end-mill prisms ->
//!   manifold booleans against the stock. Removed volume is cross-
//!   checked against an independent kernel CSG cut of the pocket prism;
//!   gouge (removal outside the intended pocket) is reported separately.
//! - `feeds`: spindle/feed numbers computed from published cutting-data
//!   formulas and charts (Harvey Tool general machining guidelines),
//!   clamped to machine limits — see docs/CAM.md citations.
//!
//! Toolpaths bind M6 named faces: the pocket top comes from a
//! `Surface::Plane` record in `Solid::table` resolved by face key, so a
//! parameter edit that moves the top face moves the whole toolpath, and
//! a rename cannot break the binding (keys embed feature ids).

pub mod feeds;
pub mod gcode;
pub mod plan;
pub mod sim;

/// CAM planning/evaluation failure, flattened to a message.
#[derive(Clone, Debug)]
pub struct CamError(pub String);

impl std::fmt::Display for CamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for CamError {}
