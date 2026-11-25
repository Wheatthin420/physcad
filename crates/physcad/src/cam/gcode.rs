//! M7: the G-code emitter and the grammar gate.
//!
//! Dialect subset (full table + citations in docs/CAM.md): GRBL v1.1,
//! modal preamble `G21 G90 G17 G94 G40`, motion `G0`/`G1` only, X/Y/Z
//! words + modal `F`, spindle `M3 S`/`M5`, non-modal `G4 P`, program
//! end `M2`. Everything else in the GRBL modal table (arcs, inch mode,
//! incremental moves, inverse-time feed, compensation, work offsets,
//! canned cycles) is NOT emitted and is rejected by both `validate`
//! here and `sim::replay`.
//!
//! The HARD COMPAT RULE lives in this file's test discipline: the
//! emitter does not verify its own output. `validate` is an independent
//! grammar gate written against the dialect spec (word sets, line
//! length vs grbl's LINE_BUFFER_SIZE, travel bounds, modal sanity),
//! while the structural parse is done by the third-party `gcode` crate
//! in tests and in `sim::replay`.

use super::CamError;

/// Machine limits a program must respect. Values are inputs, not dialect
/// claims; GRBL's own settings ($110-$112 max rate, $130-$132 max
/// travel) configure the real controller the same way (docs/CAM.md).
#[derive(Clone, Copy, Debug)]
pub struct MachineSpec {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
    pub z_min: f64,
    pub z_max: f64,
    /// max cutting feed, mm/min ($110-$112 class limit)
    pub max_feed: f64,
    /// spindle speed we command with M3 S (must be <= $30 max)
    pub spindle_rpm: f64,
    /// spindle spin-up dwell in seconds (G4 P)
    pub spindle_dwell_s: f64,
}

/// Typed motion IR: one linear move (mission rule: semantics as data).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Motion {
    Rapid { x: f64, y: f64, z: f64 },
    Cut { x: f64, y: f64, z: f64 },
}

impl Motion {
    pub fn xyz(&self) -> (f64, f64, f64) {
        match *self {
            Motion::Rapid { x, y, z } | Motion::Cut { x, y, z } => (x, y, z),
        }
    }
    pub fn is_cut(&self) -> bool {
        matches!(self, Motion::Cut { .. })
    }
}

/// Format one mm coordinate: sign, 1-4 integer digits, exactly 3
/// decimals (GRBL parses floating point words; our grammar gate below
/// pins this exact shape).
fn coord(v: f64) -> String {
    let v = if v == 0.0 { 0.0 } else { v }; // kill "-0.000"
    format!("{v:.3}")
}

/// Emit a full program: modal preamble, spindle spin-up + dwell, the
/// move list (cut moves carry the modal feed word explicitly on every
/// line — legal and self-documenting), spindle off, clear Z, end.
pub fn emit(moves: &[Motion], feed: f64, m: &MachineSpec, header: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!("(physcad M7 pocket | {header})\n"));
    // Modal preamble. G54 is GRBL's power-on default work offset and is
    // intentionally NOT emitted (reset state per docs/CAM.md citation).
    out.push_str("G21 G90 G17 G94 G40\n");
    out.push_str(&format!("M3 S{}\n", m.spindle_rpm.round() as u32));
    if m.spindle_dwell_s > 0.0 {
        out.push_str(&format!("G4 P{:.3}\n", m.spindle_dwell_s));
    }
    for (i, mv) in moves.iter().enumerate() {
        let (x, y, z) = mv.xyz();
        match mv {
            Motion::Rapid { .. } => {
                out.push_str(&format!("G0 X{} Y{} Z{}\n", coord(x), coord(y), coord(z)));
            }
            Motion::Cut { .. } => {
                let f = format!("F{feed:.3}");
                // Only word out an axis that changed; modal position carries
                // the rest (legal RS-274 wordless repetition).
                let mut line = String::from("G1");
                let prev = if i == 0 {
                    None
                } else {
                    Some(moves[i - 1].xyz())
                };
                match prev {
                    Some((px, py, pz)) => {
                        if x != px {
                            line.push_str(&format!(" X{}", coord(x)));
                        }
                        if y != py {
                            line.push_str(&format!(" Y{}", coord(y)));
                        }
                        if z != pz {
                            line.push_str(&format!(" Z{}", coord(z)));
                        }
                    }
                    None => {
                        line.push_str(&format!(" X{} Y{} Z{}", coord(x), coord(y), coord(z)));
                    }
                }
                out.push_str(&format!("{line} {f}\n"));
            }
        }
    }
    let (_, _, z_top) = moves
        .iter()
        .rev()
        .find(|mv| matches!(mv, Motion::Rapid { .. }))
        .map(|mv| mv.xyz())
        .unwrap_or((0.0, 0.0, m.z_max));
    out.push_str("M5\n");
    out.push_str(&format!("G0 Z{}\n", coord(z_top.max(0.0))));
    out.push_str("M2\n");
    out
}

/// Independent grammar gate on emitted text. This is OUR dialect subset
/// check (docs/CAM.md): every line must be one of the program shapes we
/// allow, within machine travel and GRBL's 80-byte line buffer
/// (LINE_BUFFER_SIZE, grbl/protocol.h). It deliberately does not share
/// code with `emit` or with the `gcode` crate the tests use.
#[derive(Clone, Debug, Default)]
pub struct ProgramStats {
    pub lines: usize,
    pub max_line_bytes: usize,
    pub cuts: usize,
    pub rapids: usize,
    pub feed_seen: bool,
}

pub fn validate(text: &str, m: &MachineSpec) -> Result<ProgramStats, Vec<String>> {
    let mut errs = Vec::new();
    let mut st = ProgramStats::default();
    let mut modal_units = false;
    let mut modal_dist = false;
    let mut modal_plane = false;
    let mut modal_feedmode = false;
    let mut modal_crc = false;
    let mut saw_m3 = false;
    let mut saw_m5 = false;
    let mut saw_m2 = false;
    let mut pos = (0.0_f64, 0.0_f64, 0.0_f64);
    let mut line_no = 0usize;
    let mut pending_f = None; // F value from the current line

    for raw in text.lines() {
        line_no += 1;
        let bytes = raw.len();
        if bytes == 0 {
            continue;
        }
        if bytes > st.max_line_bytes {
            st.max_line_bytes = bytes;
        }
        // GRBL's line buffer is LINE_BUFFER_SIZE=80 incl. NUL: usable 79.
        if bytes > 79 {
            errs.push(format!(
                "line {line_no}: {bytes} bytes > 79 (grbl LINE_BUFFER_SIZE=80 incl. NUL)"
            ));
        }
        // comment handling: only (...) allowed, whole-line only at start
        let body = if raw.starts_with('(') {
            if !raw.ends_with(')') {
                errs.push(format!("line {line_no}: unterminated comment"));
            }
            st.lines += 1;
            continue;
        } else if raw.contains('(') || raw.contains(';') {
            errs.push(format!("line {line_no}: trailing comments not emitted"));
            continue;
        } else {
            raw
        };
        st.lines += 1;

        let mut words: Vec<(char, String)> = Vec::new();
        for tok in body.split_whitespace() {
            let mut ch = tok.chars();
            let Some(letter) = ch.next() else { continue };
            let rest: String = ch.collect();
            if !letter.is_ascii_alphabetic() || rest.is_empty() {
                errs.push(format!("line {line_no}: bad token {tok:?}"));
                continue;
            }
            words.push((letter.to_ascii_uppercase(), rest));
        }
        if words.is_empty() {
            errs.push(format!("line {line_no}: no words"));
            continue;
        }
        // canonical emission order: G before M before X Y Z F/S/P
        let order = |c: char| match c {
            'G' => 0,
            'M' => 1,
            'X' => 2,
            'Y' => 3,
            'Z' => 4,
            'F' => 5,
            'S' => 6,
            'P' => 7,
            _ => 9,
        };
        let keys: Vec<char> = words.iter().map(|w| w.0).collect();
        for w in words.windows(2) {
            if order(w[0].0) > order(w[1].0) {
                errs.push(format!(
                    "line {line_no}: words out of canonical order {keys:?}"
                ));
                break;
            }
        }
        // at most one MOTION code per line (modal Gs may repeat freely)
        let motion_count = words
            .iter()
            .filter(|(k, v)| {
                *k == 'G' && matches!(v.parse::<u32>().ok(), Some(0) | Some(1) | Some(2) | Some(3))
            })
            .count();
        if motion_count > 1 {
            errs.push(format!("line {line_no}: more than one motion code"));
        }
        let mut dup: [bool; 256] = [false; 256];
        for (k, _) in &words {
            if matches!(k, 'X' | 'Y' | 'Z' | 'F' | 'S' | 'P') {
                if dup[*k as usize] {
                    errs.push(format!("line {line_no}: duplicate word {k}"));
                }
                dup[*k as usize] = true;
            }
        }

        let num = |s: &str, ok: &mut bool| -> f64 {
            s.parse::<f64>()
                .ok()
                .filter(|v| v.is_finite())
                .unwrap_or_else(|| {
                    *ok = false;
                    0.0
                })
        };

        for (k, v) in &words {
            match *k {
                'G' => {
                    let code = v.parse::<f64>().ok().filter(|c| {
                        (0..=4).contains(&(*c as i32))
                            || (17..=21).contains(&(*c as i32))
                            || (40..=41).contains(&(*c as i32))
                            || (49..=54).contains(&(*c as i32))
                            || (80..=95).contains(&(*c as i32))
                    });
                    match code.map(|c| c as u32) {
                        Some(0) | Some(1) => {}
                        Some(4) => {}
                        Some(21) => modal_units = true,
                        Some(90) => modal_dist = true,
                        Some(17) => modal_plane = true,
                        Some(94) => modal_feedmode = true,
                        Some(40) => modal_crc = true,
                        Some(other) => errs.push(format!(
                            "line {line_no}: G{other} outside emitted subset (0 1 4 17 21 40 90 94)"
                        )),
                        None => errs.push(format!("line {line_no}: bad G word {v:?}")),
                    }
                }
                'M' => match v.parse::<u32>().ok() {
                    Some(3) => {
                        if saw_m3 {
                            errs.push(format!("line {line_no}: M3 issued twice"));
                        }
                        saw_m3 = true;
                    }
                    Some(5) => saw_m5 = true,
                    Some(2) => saw_m2 = true,
                    Some(other) => {
                        errs.push(format!("line {line_no}: M{other} outside subset (3 5 2)"))
                    }
                    None => errs.push(format!("line {line_no}: bad M word {v:?}")),
                },
                'X' | 'Y' | 'Z' => {
                    if !is_coord(v) {
                        errs.push(format!("line {line_no}: bad {k} word {v:?}"));
                    } else {
                        let mut ok = true;
                        let val = num(v, &mut ok);
                        if ok {
                            match *k {
                                'X' => pos.0 = val,
                                'Y' => pos.1 = val,
                                _ => pos.2 = val,
                            }
                        }
                    }
                }
                'F' => {
                    if !is_coord(v) {
                        errs.push(format!("line {line_no}: bad F word {v:?}"));
                    } else {
                        let mut ok = true;
                        let f = num(v, &mut ok);
                        if ok {
                            if f <= 0.0 || f > m.max_feed {
                                errs.push(format!(
                                    "line {line_no}: feed {f} outside (0, {}]",
                                    m.max_feed
                                ));
                            }
                            st.feed_seen = true;
                            pending_f = Some(f);
                        }
                    }
                }
                'S' => {
                    let bad = match v.parse::<f64>() {
                        Ok(s) => !(0.0 < s && s <= 1e6),
                        Err(_) => true,
                    };
                    if bad {
                        errs.push(format!("line {line_no}: bad S word {v:?}"));
                    }
                }
                'P' => {
                    let bad = match v.parse::<f64>() {
                        Ok(p) => !(0.0..=3600.0).contains(&p),
                        Err(_) => true,
                    };
                    if bad {
                        errs.push(format!("line {line_no}: bad P word {v:?}"));
                    }
                }
                other => errs.push(format!("line {line_no}: word {other} not in subset")),
            }
        }

        // motion lines must respect travel bounds and modal feed
        if keys.contains(&'G') {
            let gm = words.iter().find(|w| w.0 == 'G').unwrap();
            let code = gm.1.parse::<u32>().unwrap_or(99);
            if code == 0 || code == 1 {
                st.lines += 0;
                if code == 1 {
                    st.cuts += 1;
                } else {
                    st.rapids += 1;
                    if words.iter().any(|w| w.0 == 'F') {
                        errs.push(format!("line {line_no}: G0 must not carry F (our dialect)"));
                    }
                }
                if !(m.x_min <= pos.0
                    && pos.0 <= m.x_max
                    && m.y_min <= pos.1
                    && pos.1 <= m.y_max
                    && m.z_min <= pos.2
                    && pos.2 <= m.z_max)
                {
                    errs.push(format!(
                        "line {line_no}: position {pos:?} outside machine travel"
                    ));
                }
                if code == 1 && pending_f.is_none() && !st.feed_seen {
                    errs.push(format!(
                        "line {line_no}: first G1 without F word (F resets to 0 on GRBL reset)"
                    ));
                }
                let _ = pending_f.take();
            }
        }
    }
    if !(modal_units && modal_dist && modal_plane && modal_feedmode && modal_crc) {
        errs.push(format!(
            "modal preamble incomplete: units={modal_units} dist={modal_dist} plane={modal_plane} feedmode={modal_feedmode} crc={modal_crc}"
        ));
    }
    if !saw_m3 || !saw_m5 {
        errs.push("spindle must be started (M3) and stopped (M5)".to_string());
    }
    if !saw_m2 {
        errs.push("program must end with M2".to_string());
    }
    if errs.is_empty() { Ok(st) } else { Err(errs) }
}

/// coordinate word grammar: optional minus, at most 4 int digits,
/// exactly 3 decimals.
fn is_coord(s: &str) -> bool {
    let s = s.strip_prefix('-').unwrap_or(s);
    match s.split_once('.') {
        Some((i, f)) => {
            !i.is_empty()
                && i.len() <= 4
                && f.len() == 3
                && i.bytes().all(|b| b.is_ascii_digit())
                && f.bytes().all(|b| b.is_ascii_digit())
        }
        None => false,
    }
}

pub fn emit_checked(
    moves: &[Motion],
    feed: f64,
    m: &MachineSpec,
    header: &str,
) -> Result<String, CamError> {
    let text = emit(moves, feed, m, header);
    match validate(&text, m) {
        Ok(_) => Ok(text),
        Err(errs) => Err(CamError(errs.join("\n"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> MachineSpec {
        MachineSpec {
            x_min: -100.0,
            x_max: 100.0,
            y_min: -100.0,
            y_max: 100.0,
            z_min: -50.0,
            z_max: 50.0,
            max_feed: 1000.0,
            spindle_rpm: 9000.0,
            spindle_dwell_s: 0.5,
        }
    }

    fn prog(moves: &[Motion]) -> String {
        emit(moves, 800.0, &spec(), "unit test")
    }

    #[test]
    fn emitted_program_passes_the_gate() {
        let moves = [
            Motion::Rapid {
                x: 0.0,
                y: 0.0,
                z: 15.0,
            },
            Motion::Cut {
                x: 0.0,
                y: 0.0,
                z: 10.0,
            },
            Motion::Cut {
                x: 5.0,
                y: 0.0,
                z: 10.0,
            },
            Motion::Rapid {
                x: 5.0,
                y: 5.0,
                z: 15.0,
            },
        ];
        let text = prog(&moves);
        let st = validate(&text, &spec()).expect("gate must pass");
        assert_eq!(st.cuts, 2);
        assert_eq!(st.rapids, 3); // incl. the emitter's closing retract before M2
        assert!(st.max_line_bytes < 80);
        // modal word-omission: the Y word stays out of the X move line
        assert!(text.contains("G1 X5.000 F800.000"));
    }

    #[test]
    fn gate_rejects_out_of_subset_and_dangerous_words() {
        // arcs, inch mode, incremental moves, compensation, no spindle
        for bad in [
            "G21 G90 G17 G94 G40\nG2 X1.000 Y1.000 I1.000 J0.000 F100.0\nM3 S1000\nM5\nM2\n",
            "G20 G90 G17 G94 G40\nM3 S1000\nG1 X1.000 Y1.000 F100.0\nM5\nM2\n",
            "G21 G91 G17 G94 G40\nM3 S1000\nG1 X1.000 Y1.000 F100.0\nM5\nM2\n",
            "G21 G90 G17 G94 G41\nM3 S1000\nG1 X1.000 Y1.000 F100.0\nM5\nM2\n",
            "G21 G90 G17 G94 G40\nG0 X1.000 Y1.000 Z1.000 F100.0\nM3 S1000\nM5\nM2\n",
            "G21 G90 G17 G94 G40\nM3 S1000\nG1 X110.000 Y1.000 F100.0\nM5\nM2\n",
        ] {
            assert!(validate(bad, &spec()).is_err(), "gate must reject: {bad}");
        }
    }

    #[test]
    fn long_lines_are_rejected() {
        let junk = "x".repeat(120);
        let bad = format!("G21 G90 G17 G94 G40\nM3 S{junk}\nG1 X1.000 F100.0\nM5\nM2\n");
        assert!(validate(&bad, &spec()).is_err());
    }
}
