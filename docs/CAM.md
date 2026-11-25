# CAM.md — M7 dialect, planner, simulator, feeds

Scope: milestone 7 draft (adaptive pocket → G-code → simulation). All
dialect statements below carry a citation fetched THIS session
(2026-11-24); anything without a citation is a statement about OUR
subset, not about any controller.

## 1. Emitted dialect subset

We emit a strict, explicitly-declared subset of GRBL v1.1's g-code
language. GRBL's modal groups and power-on defaults are read from the
`$G` section of the official gnea/grbl wiki "Grbl v1.1 Commands" page:
https://github.com/gnea/grbl/wiki/Grbl-v1.1-Commands (the `$G` reply
example `[GC:G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 S0.0 F500.0]` and the
modal-group table; bolded defaults there are: G0, G54, G17, G90,
G91.1, G94, G21, G40, G49, M0, M5, M9 — and T/S/F all reset to 0).

| Concern            | We emit                | Modal default we rely on | Citation (fetched 2026-11-24) |
|--------------------|------------------------|--------------------------|-------------------------------|
| Units              | `G21` (mm) once       | G21 is GRBL's power-on/reset default units mode | Commands wiki, modal table row "Units Mode: G20, **G21**" |
| Distance mode      | `G90` (absolute) once | G90 is the reset default | Commands wiki, "Distance Mode: **G90**, G91" |
| Motion plane       | `G17` (XY) once       | G17 is the reset default | Commands wiki, "Plane Select: **G17**, G18, G19" |
| Feed rate mode     | `G94` (mm/min) once   | G94 is the reset default | Commands wiki, "Feed Rate Mode: G93, **G94**" |
| Cutter comp        | `G40` (off) once      | G40 is the only member of the group's defaults; GRBL lists no G41/G42 | Commands wiki, "Cutter Radius Compensation: **G40**" |
| Work offset        | (nothing — G54)       | G54 is the reset default work coordinate system | Commands wiki, "Coordinate System Select: **G54**, …" |
| Motion             | `G0` rapid, `G1` linear feed | G0 is the reset motion mode | Commands wiki, "Motion Mode: **G0**, G1, …" |
| Feed word          | `F` on every `G1`     | F resets to 0, so the first G1 must carry it | Commands wiki, "active `T` … `S` … and `F` feed rate … all default to 0 upon a reset" |
| Spindle            | `M3 S…` once, `M5` once | M5 is the reset default; M3/M4/M9 coolant default M9 | Commands wiki, "Spindle State: M3, M4, **M5**" |
| Dwell              | non-modal `G4 P…` once | G4 is in GRBL's supported non-modal list | Commands wiki, "Supported Non-Modal Commands: G4, …" |
| Program end        | `M2` last             | M0/M1/M2/M30 program-mode group | Commands wiki, "Program Mode: **M0**, M1, M2, M30" |
| Comments           | one `(...)` header    | GRBL's line reader strips `(...)` comments | grbl/protocol.c lines ~114–128: "Throw away all (except EOL) comment characters… Enable comments flag" https://github.com/gnea/grbl/blob/master/grbl/protocol.c |
| Line length        | every line ≤ 79 bytes | LINE_BUFFER_SIZE = 80 incl. NUL | grbl/protocol.h lines 31–32: `#define LINE_BUFFER_SIZE 80` https://github.com/gnea/grbl/blob/master/grbl/protocol.h; config.h documents overriding it (line 450) https://github.com/gnea/grbl/blob/master/grbl/config.h |

NOT emitted (and rejected by both our grammar gate and the simulator):
G2/G3 arcs, G38.x probing, G80 canned cycles, G91/G91.1 incremental,
G20 inch mode, G93 inverse-time feed, G41/G42 compensation, G43.1/G49
tool length offsets, G54–G59 explicit, G10/G28/G30/G53/G92 non-modal,
M4, M7/M8 coolant, T tool changes, `#` variables. This is our subset;
no claim that GRBL lacks them (most it has — see the same table).

Machine parameters ($-settings class) are INPUTS to our planner:
`MachineSpec { x/y/z travel, max_feed, spindle ceiling }`. GRBL's
Configuration wiki documents the corresponding controller settings —
$110–$112 max rate (mm/min), $120–$122 acceleration, $130–$132 max
travel, $30 max spindle RPM, sample values in the $$ listing
https://github.com/gnea/grbl/wiki/Grbl-v1.1-Configuration (fetched
2026-11-24; its sample row shows 500 mm/min — a machine-specific
number, not a dialect limit; our gates run a 1000 mm/min machine).

## 2. Independent parser (hard-compat rule)

The emitter is never its own judge:

- every emitted program is parsed in tests by the third-party crate
  `gcode` 0.7.0, "A gcode parser for no-std applications" (same-named
  crates.io project by Macoy Madson et al.), license MIT OR Apache-2.0
  (crates.io `versions[0].license`, fetched 2026-11-24),
  https://crates.io/crates/gcode / https://docs.rs/gcode — a
  structural parser independent of our writer;
- `sim::replay` builds all machine motion ONLY from that parser's
  AST (blocks → G/M codes + word addresses), so the simulator reads
  the program the way an independent reader would;
- `cam::gcode::validate` is OUR dialect grammar gate (separate
  implementation, checks word order/duplication, coordinate grammar,
  travel bounds, line bytes, modal preamble completeness);
- candidates considered and rejected: `nc` on crates.io is now a
  syscalls crate (xushaohua/nc — description "Access system calls
  directly"), NOT the old CNC parser the name suggests; `g-code`
  0.7.0 (sameer/g-code, MIT OR Apache-2.0) is a valid peg-grammar
  alternative — heavier; `rc_gcode`/`gcode-nom`/`async-gcode` left
  unexercised.

## 3. Planner (docs of record for cam/plan.rs)

Contour-parallel ("adaptive": rings follow the part, islands included)
offset pocketing over a rasterized heightfield of the region:

1. rasterize region (even-odd polygon, `pixel` mm);
2. exact squared Euclidean distance transform (Felzenszwalb–Huttenlocher);
3. cutter-center levels at `t = r + k(2r − stepover)` + core level;
   bounded coverage-driven refinement inserts a corrective level when
   a shrinking inner contour stops wrapping a strip (reflex corners);
4. marching squares, inside-left segments (outer rings CCW, island
   rings CW by construction), RDP simplification to `chord` mm;
5. linking: straight bridges ONLY if full tool clearance (EDT ≥ r −
   2·pixel) holds along sampled samples; otherwise retract+plunge
   (G0/G1 pairs) — a bridge that would graze wall or island is a
   geometry decision, never a silent gouge;
6. depth passes at `depth_per_pass`, plunges at ring points (tangent
   to the wall), rapids above `z_top + z_clear`;
7. self-certification: every pixel a full r-disc can reach (distance
   to the eroded set {t ≥ r} ≤ r + pixel) must lie within r + 2·pixel
   of a cut path, or planning fails; pixels NO r-disc can reach
   (sharp convex corners) are reported as `rest_area_mm2` — rest
   roughing with a smaller tool is deferred, and the sim cross-check
   below counts that leftover honestly.

Face binding (M6): the pocket's `z_top` comes from the named face
record (`Solid::table`, key `F{feature}:{tag}`); a parameter edit that
moves the face moves the whole toolpath (gate
`gate_toolpath_binding_survives_param_edit_and_rename`), and keys
embed feature ids, so renames cannot break a CAM reference.

Known deferred: trochoidal/high-efficiency roughing, helical entry,
ramp-in, toolholder/holder-geometry collision, feeds scaled to radial
engagement, multiple tools/rest tools, G2/G3 output (arcs are
in-domain for GRBL but not for M7's planner).

## 4. Simulator + cross-check bound

`cam::sim::replay` (gcode crate parse → modal state machine → flat
end mill swept prisms (stadium cross-sections, 24-step caps, prism
span tip→tip+length) → balanced manifold union → booleans against the
stock manifold). Ground truth: stock cut by the pocket prism extruded
from the exact sketch region polygon; removals compared per volume.

Stated bound (asserted in `tests/m7_gates.rs`):
- |V_sim − V_csg| ≤ 3 % of V_csg (dominated by raster `pixel=0.05 mm`
  + ring chord `0.05 mm` wall quantization along the ~0.5 m perimeter);
- gouge (removal outside the pocket prism) ≤ 0.5 % of V_csg;
- uncut (prism volume never reached) ≤ 2.5 % of V_csg — includes the
  corner rest material above.

Measured 2026-11-24 (gate run, printed by the test):
`SIM pocket_csg=6983.4 mm3 sim_removed=6955.9 mm3 diff=0.3942%
gouge=28.21 mm3 (0.4039%) uncut=55.74 mm3 cut_xy=212 cut_z=8
rapids=10 lines=236 coverage_excess=0.0480 mm rest_area=9.66 mm2` —
7.6× margin on the volume bound. The unit-level sim gate
(`straight_cut_matches_boolean_ground_truth`) pins the same machinery
against an analytic stadium volume within 2 % and near-zero gouge.

## 5. Feeds — computed, not guessed

`cam/feeds.rs` implements exactly the published relations from Harvey
Tool's General Machining Guidelines
https://www.harveytool.com/resources/general-machining-guidelines
(fetched 2026-11-24): for carbide end mills on Aluminum
2024-T4/T6, 2014, 6061-T6/T651, 7075-T6: `SFM 800-1500`, chip load
(IPT) `.001–.007` by cutter diameter (1/4" column: `.002`), with
`RPM = (3.82 x SFM) / D` and `IPM = RPM x IPT x T`.

Gate numbers (6 mm ≈ 1/4" 2-flute, low end of the band):
`rpm_raw = 3.82*800/(6/25.4) = 12937.6` → capped by the machine's
spindle ceiling 10000 → `feed_raw = 10000 * 0.002 * 2 * 25.4 =
1016.0 mm/min` → capped by the machine feed ceiling 1000 → effective
chip load `1000 / (10000 × 2) = 0.05 mm/tooth = 0.00197 in` — still
inside the chart's .002-class neighborhood at 1/4" (0.05 mm/tooth
with a 10 mm cutter also matches the published .05–.08 mm/tooth
starting ranges). The spindle/feed ceilings are machine settings
($30, $110–$112 class), declared in `MachineSpec`, NOT taken as
dialect constants. Radial-engagement adjustment (chip-thinning at
full-width slotting) is deferred; the gates note it.

## 6. Output discipline

Nothing here ever runs a machine. The DoD's human gate stands: a
person reviews the G-code text (docs/m7-toolpath.png renders the
toolpath for eyeball checks; every number above is reproducible by
`cargo test -p physcad --test m7_gates -- --nocapture`).
