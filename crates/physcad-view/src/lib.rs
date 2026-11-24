//! Viewer shell: interactive eframe window, or headless PNG capture.
//!
//! `physcad-view`                      -> window (needs a display server)
//! `physcad-view --capture out.png`    -> offscreen render, no display

pub mod app;
pub mod renderer;
pub mod scene;

pub use renderer::{RenderReport, capture_png};
pub use scene::sample_scene;
