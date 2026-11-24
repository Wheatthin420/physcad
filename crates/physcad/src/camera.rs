//! Deterministic orbit-camera math (shared by the offscreen renderer and
//! the interactive eframe shell). Y-up, right-handed, OpenGL-style clip.

use glam::{Mat4, Vec3};

/// Orbit camera defined by target point, distance, yaw (around +Y) and
/// pitch (elevation above the XZ plane), both in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrbitCamera {
    pub target: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub fov_y: f32,
    pub near: f32,
    pub far: f32,
}

pub const MIN_DISTANCE: f32 = 0.05;
pub const MAX_PITCH: f32 = 1.5; // ~86 deg, stops at the poles

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Vec3::ZERO,
            distance: 9.0,
            yaw: 0.9,
            pitch: 0.35,
            fov_y: 50f32.to_radians(),
            near: 0.05,
            far: 200.0,
        }
    }
}

impl OrbitCamera {
    /// Unit direction from target toward the eye.
    pub fn direction(&self) -> Vec3 {
        let (s, c) = (self.pitch.sin(), self.pitch.cos());
        Vec3::new(c * self.yaw.cos(), s, c * self.yaw.sin())
    }

    pub fn eye(&self) -> Vec3 {
        self.target + self.distance * self.direction()
    }

    pub fn view(&self) -> Mat4 {
        look_at_rh(self.eye(), self.target, Vec3::Y)
    }

    pub fn projection(&self, aspect: f32) -> Mat4 {
        perspective_rh(self.fov_y, aspect, self.near, self.far)
    }

    pub fn view_projection(&self, aspect: f32) -> Mat4 {
        self.projection(aspect) * self.view()
    }

    /// Horizontal/vertical orbit from a drag delta in *pixels*.
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        const SENS: f32 = 0.006;
        self.yaw -= dx * SENS;
        self.pitch = (self.pitch + dy * SENS).clamp(-MAX_PITCH, MAX_PITCH);
        // Normalize yaw into [0, 2pi) so comparisons stay stable over time.
        self.yaw = self.yaw.rem_euclid(std::f32::consts::TAU);
    }

    /// Zoom from a scroll delta (positive = toward target).
    pub fn zoom(&mut self, delta: f32) {
        self.distance = (self.distance * (1.0 - delta * 0.001)).max(MIN_DISTANCE);
    }
}

/// Right-handed look-at (OpenGL convention: camera looks down -Z).
/// glam 0.34 dropped its camera helpers; we own these matrices now, which
/// also lets us add an ortho variant later for drawing sheets.
fn look_at_rh(eye: Vec3, center: Vec3, up: Vec3) -> Mat4 {
    let f = (center - eye).normalize();
    let s = f.cross(up).normalize();
    let u = s.cross(f);
    Mat4::from_cols_array_2d(&[
        [s.x, u.x, -f.x, 0.0],
        [s.y, u.y, -f.y, 0.0],
        [s.z, u.z, -f.z, 0.0],
        [-s.dot(eye), -u.dot(eye), f.dot(eye), 1.0],
    ])
}

/// Right-handed perspective into OpenGL clip space (z in [-1, 1]).
fn perspective_rh(fov_y: f32, aspect: f32, near: f32, far: f32) -> Mat4 {
    let f = 1.0 / (fov_y * 0.5).tan();
    let nf = near - far;
    Mat4::from_cols_array_2d(&[
        [f / aspect, 0.0, 0.0, 0.0],
        [0.0, f, 0.0, 0.0],
        [0.0, 0.0, (far + near) / nf, -1.0],
        [0.0, 0.0, (2.0 * far * near) / nf, 0.0],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn yaw_zero_places_eye_on_plus_x() {
        let cam = OrbitCamera {
            yaw: 0.0,
            pitch: 0.0,
            distance: 5.0,
            ..Default::default()
        };
        let eye = cam.eye();
        assert!((eye - Vec3::new(5.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn pitch_90_places_eye_on_plus_y() {
        let cam = OrbitCamera {
            yaw: 0.0,
            pitch: FRAC_PI_2,
            distance: 4.0,
            ..Default::default()
        };
        let eye = cam.eye();
        assert!((eye - Vec3::new(0.0, 4.0, 0.0)).length() < 1e-4);
    }

    #[test]
    fn yaw_90_moves_eye_to_plus_z() {
        let cam = OrbitCamera {
            yaw: FRAC_PI_2,
            pitch: 0.0,
            distance: 3.0,
            ..Default::default()
        };
        let eye = cam.eye();
        assert!((eye - Vec3::new(0.0, 0.0, 3.0)).length() < 1e-4);
    }

    #[test]
    fn view_transform_sends_eye_to_origin() {
        let cam = OrbitCamera::default();
        let v = cam.view().transform_point3(cam.eye());
        assert!(v.length() < 1e-4);
    }

    #[test]
    fn orbit_clamps_pitch_and_keeps_distance() {
        let mut cam = OrbitCamera::default();
        let d0 = cam.distance;
        for _ in 0..1000 {
            cam.orbit(0.0, -100.0); // spin toward the pole
        }
        assert!(cam.pitch <= MAX_PITCH + 1e-6);
        assert_eq!(cam.distance, d0);
        assert!((0.0..std::f32::consts::TAU).contains(&cam.yaw));
    }

    #[test]
    fn zoom_clamps_at_min_distance() {
        let mut cam = OrbitCamera::default();
        for _ in 0..1000 {
            cam.zoom(100.0);
        }
        assert!(cam.distance >= MIN_DISTANCE);
    }

    #[test]
    fn point_at_target_projects_to_screen_center() {
        let cam = OrbitCamera {
            target: Vec3::X * 2.0,
            ..Default::default()
        };
        let vp = cam.view_projection(16.0 / 9.0);
        let clip = vp.project_point3(cam.target);
        assert!(
            clip.truncate().length() < 1e-3,
            "target should land at NDC (0,0), got {clip}"
        );
        // target is in front of the camera: right-handed view space looks down -Z
        let view_z = cam.view().transform_point3(cam.target).z;
        assert!(
            view_z < 0.0,
            "target should be in front of the camera, view.z={view_z}"
        );
    }

    #[test]
    fn default_camera_is_sane() {
        let cam = OrbitCamera::default();
        assert!(cam.far > cam.near);
        assert!((0.0..PI).contains(&cam.fov_y));
    }
}
