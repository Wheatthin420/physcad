//! M7 feeds: spindle speed and table feed COMPUTED from published
//! formulas + chart values — the numbers and their source live in
//! docs/CAM.md §feeds. No hard-coded magic feed rate exists in this
//! project; every F word traces through this function from cited inputs.

use super::gcode::MachineSpec;

#[derive(Clone, Copy, Debug)]
pub struct SideCutInputs {
    /// cutting speed, surface feet per minute (chart value)
    pub sfm: f64,
    /// chip load, inches per tooth (chart value)
    pub chip_load_in: f64,
    /// cutter diameter, inches
    pub dia_in: f64,
    /// number of flutes
    pub teeth: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FeedCalc {
    pub rpm_raw: f64,
    pub rpm_used: f64,
    pub feed_raw_mm: f64,
    pub feed_used_mm: f64,
    /// effective chip load after machine clamps, mm/tooth
    pub chip_used_mm: f64,
    pub clamped_spindle: bool,
    pub clamped_feed: bool,
}

/// Harvey Tool general guidelines: `RPM = (3.82 x SFM) / D` and
/// `IPM = RPM x IPT x T` (inches); converted to mm and clamped to the
/// machine's $30-class spindle ceiling and $110-class feed ceiling.
pub fn compute(i: &SideCutInputs, m: &MachineSpec) -> FeedCalc {
    let rpm_raw = 3.82 * i.sfm / i.dia_in;
    let rpm_used = rpm_raw.min(m.spindle_rpm);
    let feed_raw_in = rpm_used * i.chip_load_in * i.teeth;
    let feed_raw_mm = feed_raw_in * 25.4;
    let feed_used_mm = feed_raw_mm.min(m.max_feed);
    let chip_used_in = if rpm_used * i.teeth > 0.0 {
        feed_used_mm / 25.4 / (rpm_used * i.teeth)
    } else {
        0.0
    };
    FeedCalc {
        rpm_raw,
        rpm_used,
        feed_raw_mm,
        feed_used_mm,
        chip_used_mm: chip_used_in * 25.4,
        clamped_spindle: rpm_used < rpm_raw,
        clamped_feed: feed_used_mm < feed_raw_mm,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formulas_match_harvey_tables_arithmetically() {
        // 1/4" 2-flute carbide, 6061-T6 at the LOW end of the chart's
        // 800-1500 SFM band, chip load .002" per the 1/4" column.
        let m = MachineSpec {
            x_min: -100.0,
            x_max: 100.0,
            y_min: -100.0,
            y_max: 100.0,
            z_min: -50.0,
            z_max: 50.0,
            max_feed: 1000.0,
            spindle_rpm: 10000.0,
            spindle_dwell_s: 0.5,
        };
        let c = compute(
            &SideCutInputs {
                sfm: 800.0,
                chip_load_in: 0.002,
                dia_in: 6.0 / 25.4,
                teeth: 2.0,
            },
            &m,
        );
        // RPM = 3.82*800/(6/25.4) = 12945.3...
        let approx = |a: f64, b: f64| (a - b).abs() < 1e-6 * b.max(1.0);
        assert!(approx(c.rpm_raw, 3.82 * 800.0 / (6.0 / 25.4)));
        // spindle ceiling binds, then recomputed feed slightly exceeds the
        // 1000 mm/min ceiling -> feed clamp + chip-load re-derivation
        assert!(c.clamped_spindle);
        assert!(approx(c.rpm_used, 10000.0));
        assert!(approx(c.feed_raw_mm, 10000.0 * 0.002 * 2.0 * 25.4));
        assert!(c.clamped_feed);
        assert!(approx(c.feed_used_mm, 1000.0));
        // chip_used_mm must equal feed/(rpm*teeth) in mm: 1000/20000 = 0.05
        assert!(approx(c.chip_used_mm, 0.05));
    }
}
