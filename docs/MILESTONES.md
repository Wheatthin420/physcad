# MILESTONES.md — status per rung (ladder defined in physcad-mission skill)

| # | Milestone | Status | Date | Evidence |
|---|---|---|---|---|
| 1 | SURVEY.md field survey + shortlist | DONE | 2026-11-24 | docs/SURVEY.md; all licenses read from project LICENSE files + crates.io API same day |
| 2 | Viewer shell (wgpu + egui, orbit, primitives, screenshot) | DONE | 2026-11-24 | crates/physcad + crates/physcad-view; docs/m2-capture.png (1280x720, llvmpipe Cpu/Vulkan adapter, sha256 in commit msg body); LICENSES.md opened |
| 3 | Mesh kernel slice (manifold3d CSG → STL, params+expressions) | DONE | 2026-11-24 | crates/physcad {expr,tess,csg,model,stl}.rs + src/bin/physcad.rs; crates/physcad/tests/m3_gates.rs (7 analytic volume gates + mass gate + bit-stability, incl. torus); crates/physcad-view/tests/csg_capture.rs (headless render of CSG result incl. kernel torus + exact-polyhedral volume check); examples/bore-plate.pcm + examples/torus-plate.pcm; docs/m3-csg.png; evidence below |
| 4 | Sketches + constraint solver (own LM; slvs blocked GPL-3.0) | DONE | 2026-11-24 | crates/physcad/src/sketch/{mod,solver}.rs (dual-number exact Jacobians, dense LM, Jacobi-eigen rank/DOF, Gram-Schmidt named redundancy) + .pcm sketch grammar in model.rs + examples/sketch-bracket.pcm + crates/physcad/tests/sketch_gates.rs (10 gates, residuals below) + crates/physcad-view/tests/m4_sketch_capture.rs (headless wgpu render of solved sketch; docs/m4-sketch.png). Developed on branch `m4-sketch` (concurrent M3-torus session held the tree), fast-forward merged to main 2026-11-24 |
| 5 | Extrude/revolve from closed sketch loops | DONE | 2026-11-24 | crates/physcad/src/sketch/loops.rs (weld+walk+nesting) + src/sweep.rs (own bridge+ear-clip cap triangulator, extrude/revolve mesh builders) + Mesh64::closed_manifold_check + GeoNode::Extrude/Revolve + .pcm grammar + examples/sketch-plate.pcm (solved sketch -> extrude -> CSG cut) + crates/physcad/tests/m5_gates.rs (11 gates; exact-polyhedral devs 0.0..1.8e-11, torus-poly rel 5.7e-3 < 6e-3 bound w/ 4x convergence) + crates/physcad-view/tests/m5_capture.rs (headless render, bore see-through probe, STL round trip; docs/m5-extrude-csg.png). Developed on branch `m5-sweep` (worktree ~/physcad-m5; concurrent repair session held main), merged to main. B-rep migration stays a documented decision point. Evidence below |
| 6 | Timeline data model + stable topology naming | DONE | 2026-11-24 | crates/physcad/src/timeline.rs (FeatureId + Timeline + insert_at/rename edit ops) + src/naming.rs (analytic FaceRecords per feature: Plane/Cylinder/Sphere/Cone/Torus; triangle re-identification + component grouping) + GeoNode operands upgraded positional usize -> FeatureId + `NAME@K` id pins in .pcm + Solid.faces readout in CLI. Mesh-based continuity heuristic, no B-rep (decision + algorithm + failure modes FM-1..FM-7 in docs/NAMING.md). crates/physcad/tests/m6_gates.rs: 12 gates incl. the DoD re-bind suite (param edit w60->80: same 7 keys, +x centroid 30->40, bore wall fixed; mid-tree rename bore->hole: keys AND centroids identical; mid-tree insertion: keys identical bit-identical centroids; extrude walls named by sketch entity; revolve cyl/plane/cone families; 180-deg cap coincidence pinned as deterministic FM-3). Evidence below |
| 7 | CAM draft (adaptive pocket → GRBL-dialect G-code + sim) | not started | — | — |

Key survey findings that changed the ladder:
- slvs = GPL-3.0; M4 proceeds with own LM solver unless user approves GPL.
- `csg` crate (old M3 fallback) does not exist on crates.io; manifold3d (Apache-2.0
  OR MIT) is the M3 kernel, period.
- First real license gate moves to M5+ (B-rep migration): monstertruck (Apache-2.0)
  vs OCCT (LGPL-2.1+exception, only needed for STEP in v2).

## M3 evidence (all commands run on this box, 2026-11-24)

Gates: `cargo fmt` clean; `cargo clippy -j 3 --all-targets` -> 0 warnings;
`cargo test -j 3` -> 43 lib + 8 gate + 3 view lib + 4 capture/integration
tests, 0 failed (post-merge tree with M4; re-run twice, green both; one
pre-rerun flake `cut_box_minus_torus_cavity_analytic` FAILED once in the
first post-merge full-suite run with no captured panic text — all 8 gates
green on the same binary in three subsequent runs; hypothesis: transient
kernel allocation rejection under memory pressure with the inference
engine holding ~59% RAM; re-run is the mitigation).

Analytic volume gates (`cargo test -j 3 -p physcad --test m3_gates -- --nocapture`),
tolerance 1e-6 mm^3 absolute, kernel volume = manifold f64:

```
GATE union box+cyl:        got 4506.283185153  analytic 4506.283185307  |err| 1.540e-7
GATE cut cyl bore:         got  193.716814847  analytic  193.716814693  |err| 1.540e-7
GATE intersect box∩cyl:    got    9.424777730  analytic    9.424777961  |err| 2.310e-7
GATE union box+hemisphere: got    1.032724641  analytic    1.032724923  |err| 2.824e-7
GATE cut spherical cavity: got    0.934550718  analytic    0.934550153  |err| 5.647e-7
GATE torus:                got    0.021318078  analytic    0.021318346  |err| 2.675e-7
GATE cut box-torus cavity: got    0.978681922  analytic    0.978681654  |err| 2.675e-7
GATE mass@7.85:            got    0.000167347  analytic    0.000167349  |err| 2.100e-9  (g)
```

Math: cylinders = inscribed 16384-gon prisms (error budget V·(2π/n)²/6 ≈
1.5e-7 — measured matches); spheres = UV-polyhedra r=1/4 at 1024×1024
(measured ≈2.3× the naive (π²/m²+4π²/3n²)/6 estimate, still 1.8× margin);
boxes exact. Tori (ring R=0.3, tube r=0.06, 1024×1024, analytic 2π²Rr²)
show textbook quadratic convergence — measured |err| 4.28e-6 → 1.07e-6 →
2.675e-7 as segments double (4.0x drop per step, asserted in
tess::torus64_error_scaling_matches_quadratic). Demo-part cross-check
(csg_capture test) matches its exact polyhedral closed form (prismoid sum)
at dev 0.000e0.

Genus conventions verified against manifold (encoded in tests):
through-hole -> +1; an extra closed SHELL contributes −chi_shell/2, so a
spherical cavity (chi=2) -> genus −1 total, but a TORUS cavity (chi=0)
leaves genus 0; a solid torus alone is genus 1.

Bit-stability:
- In-process: `scripted_rebuild_is_bit_stable` — two evaluates of the same
  parsed model (now including a torus + move + all three CSG ops) -> volume
  bit-identical, mesh equal, 491984 STL bytes equal.
- Cross-process: two `./target/debug/physcad build examples/bore-plate.pcm --stl …`
  runs -> both `sha256 16004736ee145b81aa7a6c3590dc855673304fd62c447d4e574a376202068ace`,
  CLI readout identical (V=28309.777414 mm^3, mass 222.2318 g @7.85 g/cm^3,
  genus 1, 5010 tris). Same sha re-verified after the torus work (no drift).
- Torus cross-process: two `build examples/torus-plate.pcm --stl …` runs ->
  both `sha256 1ecf0f21843ec770fdb6641957138279e71342403373fb7f5866a2e244aa5af9`
  (442484 bytes), readout identical (V=9903.265535 mm^3, 77.7406 g, genus 1,
  8848 tris — hand-checked: 9600 plate + 472.8 half-ring − 169.5 bore).
- STL f32 quantization, f64-summed: relative volume drift 5.3e-10.

Kernel setup: manifold3d 0.4.2 w/o default features (no TBB; single-threaded
upstream manifold v3.5.5, LICENSE verified in build tree, see LICENSES.md).
cmake 3.31.6 standalone at ~/.local/opt/cmake (no apt on box, PEP 668
blocked pip) — added to PATH via ~/.bashrc.

Genus convention (verified, tests encode it): through-hole -> +1; extra
closed shell contributes -chi/2 (spherical shell -> -1; torus shell, chi=0,
-> no change); solid torus alone -> genus 1.

Render gate (csg_capture): the scene now holds TWO kernel-built solids —
the bore-plate and `torus(8, 2, 96, 48)` (render tessellation; analytic
slop 3.6e-3 relative, the 1e-6 gates stay in m3_gates.rs). llvmpipe CPU
adapter at 640x360; pixel checks count 7913 blue (plate) + 1622 copper
(ring) off-background pixels, and the copper bbox (61x44) contains 1062
background pixels -> the ring hole sees through. docs/m3-csg.png
(sha256 91ec982c7c647698bc10cfe23defad83b13f5b5336d4695faa0fbd76e16da43b,
regenerated post-merge with the corrected cull_mode: None renderer; pixel
counts above unchanged). Vision self-check (aux.vision) confirms plate +
bore spot + dome knob + ring with visible hole; zero black px in the file
(vision's "letterbox bars" were confabulation — pixel probe wins).

Known limits after M3: no rotations yet (move only), UV-sphere poles pinch
triangle sizes, spindle tori (tube > ring radius) not gated, no units
system, ASCII STL not implemented.

## M4 evidence (all commands run on this box, 2026-11-24)

Gates: `cargo fmt` clean; `cargo clippy -j 3 --all-targets` -> 0 warnings;
`cargo test -j 3` -> 41 lib + 6 m3 + 10 sketch_gates + 3 view lib + 2 csg
+ 1 m4 capture + 1 offscreen, 0 failed. (Counts at M4 merge time; the tree
grew to 43 lib + 8 m3 after the m3-torus rebase — see M3 evidence above
for the combined post-merge totals.)

Exact solve residuals (`cargo test -j 3 -p physcad --test sketch_gates -- --nocapture`),
max |residual| per equation, convergence tol 1e-12:

```
triangle-3-4-5 (3 side lengths)      max|res| 8.882e-16  iters 6   rank 3/6  dof 3  (rigid body)
rect to axes (h/v/dims/anchor)       max|res| 1.421e-14  iters 6   rank 8/8  dof 0  exact corners
circle tangent 2 perp. lines         max|res| 8.527e-14  iters 8   rank 9/9  dof 0  center (4,4)
circle in 60-deg wedge (angle cons)  max|res| 2.138e-13  iters 9   rank 8/9  dof 1  center (4*sqrt3, 4)
over-constrained (dup distance)      max|res| 8.882e-16  iters 6   redundant: [ab_again]
contradictory sides 1-1-5            Unsolved status,   max|res| 1.000e0   no panic, honest message
coincident merge                     max|res| 1.554e-15  iters 5   rank 4/4  dof 0
line tangent arc (parabola)          max|res| 7.105e-15  iters 7   rank 9/10 dof 1  parabola dev 7.1e-15
two tangent circles (over-pinned)    max|res| 1.659e-16  iters 5   redundant: [tan]  |c1-c2| = 8.000000000000
determinism (same sketch twice)      params bit-identical, residuals bit-identical
```

.pcm integration (`./target/debug/physcad build examples/sketch-bracket.pcm`):
`br (14 entities, 13 constraints): solved: 17 eqs, 17 params, rank 17, dof 0,
max|res| 5.684e-14, 8 iters`. Parse errors carry file:line:col and name the
offending operand kind ("horizontal: line #0 is point, must be line").

Render gate (`cargo test -j 3 -p physcad-view --test m4_sketch_capture`):
llvmpipe Cpu/Vulkan, 640x360, measured px: teal curves 956, amber
fillet+arc 417, red point dots 89, x-span 195; solved sketch drawn as
flat ribbons. docs/m4-sketch.png sha256
7c64787b73887a63db8f6bd5bd63abf9f2c3616abf00d95bd00035f3d6dfe36d.

Solver design facts worth remembering:
- Exact Jacobians via dual numbers (one residual implementation serves
  value + derivative passes); 3BCAD's lambda*(1+diag(JtJ)) damping trick
  keeps rank-deficient columns solvable.
- Rank/DOF at the solution: eigen of JtJ (cyclic Jacobi), etol =
  1e-10*max(lmax,1); null eigenvectors name free params; redundant
  constraints named by greedy Gram-Schmidt over Jacobian rows.
- line-circle / line-arc tangency solved as distance^2 = radius^2:
  sign-agnostic (either side), polynomial-smooth, no |x| kink.
- Viewer finding: wgpu y-down framebuffer + our CCW-outside convention
  means `cull_mode: Front` culled the CAMERA-FACING surfaces; closed
  solids survived only via depth-tested far walls. Now `cull_mode: None`.

License gate: NO new third-party deps added for M4 (nalgebra considered,
license crates.io-verified Apache-2.0/BSD-3 — not needed; our dense LA is
~200 LoC). slvs GPL-3.0 remains blocked pending user approval.

Known limits after M4: no ellipse/spline entities; angle between arcs or
point-line distance constraints not implemented; no construction geometry;
sketch loops not yet extracted for extrude (M5); solver is dense O(n^3)
fine up to a few hundred params, component-splitting (3BCAD) when needed.

## M3 repair — torus gate red-run forensics (2026-11-24, main@aa43a19)

Observed (parent session): `cargo test -j 3` on main@aa43a19 failed
`torus_analytic_volume_mass_and_genus` with
`kernel output failed closed-manifold check (2097152 tris)` — 7/8 gates.

Could NOT reproduce on source (every command on this box, same day):
- 5/5 full m3_gates runs green (incl. the parent's exact 8-test binary), plus
  an instrumented probe replaying the exact gate path: 6/6 release + 12/12
  dev-profile rounds green.
- Probe audit of the kernel output at gate resolution, INDEPENDENT of the
  gate's own check: torus64 grid 1,048,576 verts with 0 bitwise-duplicate
  positions; manifold round trip -> 2,097,152 tris, 6,291,456 directed edges,
  0 degenerate tris, 0 edges with count != 1, 0 missing opposite pairs,
  0 f64->f32 vertex collisions.
- The Rust path (torus64 -> weld-dedup -> from_mesh_f64 -> to_mesh_f64 ->
  f32 narrow -> edge audit) is pure; manifold-csg-sys 3.5.106 builds
  single-threaded (default-features=false). Same source + same data cannot
  give different verdicts, so the red run did not have the source tree's data.

The 2^21 figure is NOT a smoking gun: 2*1024*1024 = 2,097,152 is exactly the
CORRECT triangle count for the requested 1024x1024 torus. A resolution
parameter bug (the prime suspicion) would show a different count; the wiring
model.rs -> GeoNode::Torus -> torus64 was checked argument-by-argument and is
now pinned by tests (model::torus_segment_args_wire_in_order with asymmetric
48/32 segs; gate asserts num_tri == 2*1024*1024).

Root cause classification for the single red run — environmental, one of:
1. Live memory corruption in the test process. Kernel journal the same day:
   `BUG: Bad page state in process strata` at 07:51:14, 07:51:15, 08:29:46
   (documented DKMS amdgpu 7.2.3 page-corruption issue on this box; a data
   bit-flip in the ~1GB of fresh HashMap/mesh pages breaks the edge audit
   while every count stays normal — precisely the observed signature).
2. Stale pre-merge artifacts (the documented M4 repair failure mode): the
   aa43a19 commit message itself quotes "41 lib + 8 gate + 2 view lib +
   3 capture" while main's tree measures 43 lib + 8 + 3 view lib + 4 capture
   — those numbers came from a worktree build, not the merge result.

The real SOFTWARE defect found: `evaluate` discarded the sub-check's Err text
(`closed_manifold_check().is_err()`), leaving only a raw tri count in the
failure — un-forensicable. Fixed: the error now carries the failing
sub-check string plus BOTH kernel and display triangle counts, which
distinguishes corruption (counts equal, edges unpaired) from conversion bugs
(counts differ) from geometry bugs (specific named edge) in one line.

Hardening (same commit): csg::torus_gate_resolution_stays_closed_manifold
(full kernel audit at gate resolution, exact counts + genus),
tess::torus64_gate_grid_is_welded_and_unique (1,048,576 bitwise-unique grid
verts), model::torus_segment_args_wire_in_order, gate num_tri assert.
The gate was strengthened, not weakened; 1024x1024 is proven feasible.

Sample-STL decision: artifacts/*.stl are regenerable CLI outputs, now
git-ignored (source of truth = examples/*.pcm + this file's sha256 records).
Regenerate: `cargo run --release --bin physcad -- build examples/<name>.pcm
--stl artifacts/<name>.stl`. Re-verified byte-identical to the aa43a19 records:
bore-plate  16004736ee145b81aa7a6c3590dc855673304fd62c447d4e574a376202068ace
torus-plate 1ecf0f21843ec770fdb6641957138279e71342403373fb7f5866a2e244aa5af9

Post-repair gates on main@4839fda (run in a clean worktree of that HEAD to
exclude a concurrent session's uncommitted WIP sitting in the main checkout):
`cargo fmt --check` clean; `cargo clippy -j 3 --all-targets` 0 warning lines;
`cargo test -j 3` exit 0 — 46 lib + 8 m3_gates + 10 sketch_gates + 3 view lib
+ 2 csg_capture + 1 m4 capture + 1 offscreen = 71 passed, 0 failed.

## M5 evidence (all commands run on this box, 2026-11-24, branch m5-sweep)

Gates: `cargo fmt` clean; `cargo clippy -j 3 --all-targets` -> 0 warnings;
`cargo test -j 3` -> see run log below (counts at merge).

Exact-polyhedral volume gates (`cargo test -j 3 -p physcad --test m5_gates -- --nocapture`);
the generated mesh IS the polyhedron, so bounds are 1e-9 absolute, not
tessellation slop:

```
GATE rect-extrude:          got 240.000000000   exact 240.000000000   dev 0.000e0
GATE circ64-extrude:        got 768.454380184   exact 768.454380184   dev 0.000e0
GATE bore-plate(sketch):    got 2415.162449238  exact 2415.162449238  dev 4.547e-13  (rect - 96-gon bore, chi 0, genus 1)
GATE revolve360 shell:      got 2408.869240739  exact 2408.869240739  dev 4.547e-13  ((A64(14)-A64(10))*8, chi 0)
GATE revolve90 shell:       got 603.170649820   exact 603.170649820   dev 8.072e-12  (chi 2; analytic sector 603.185789489;
                                      convergence vs continuous: e(32) 2.422e-1 -> e(128) 1.514e-2, ~16x for 4x steps)
GATE extrude+cut:           got 2367.162449238  exact 2367.162449238  dev 4.547e-13  (pocket -48 exact, genus 1)
GATE two-region extrude:    got 100.000000000   exact 100.000000000   dev 1.421e-14  (genus -1 two shells, chi 4)
GATE offset extrude:        z span [3.000, 9.000] pinned, V 240 exact
GATE torus-poly(48,48):     got 628.055170      analytic 631.654682   rel 5.699e-3  (< stated 6e-3 bound; chi 0, genus 1)
GATE torus-poly(96,96):     rel 1.427e-3                                              (4.0x drop: quadratic)
GATE sketch-plate.pcm:      V=26854.299840599   exact 26854.299840599 dev 1.819e-11 (bore sampled chord-tol -> 36-gon)
   solve from file: plate (10 entities, 9 constraints): 11 eqs, 11 params, rank 11, dof 0, max|res| 1.776e-14, 5 iters
   STL export: 8884 bytes; f64-sum reread rel vol drift 3.084e-10; cross-process sha256 e50c381b71de0685... twice
```

Closed-manifold discipline: every sweep mesh passes
`tess::Mesh64::closed_manifold_check` (edge pairing + Euler) inside
`evaluate_with` BEFORE kernel submission (KernelError names the node and
the sub-check), and `evaluate` re-checks the display mesh after kernel
round trip (M3's hardened forensics message path). Euler chars asserted
per gate: box=2, annulus extrude=0, full revolve=0, partial=2, two
shells=4.

Cap triangulator design (sweep.rs): bridge each hole into the outer
polygon with the shortest fully-clear cut traversed OUT AND BACK between
the same vertex pair (bridge terms cancel in the shoelace sum), then
ear-clip with bridge-corner ids never clipped unless nothing else is
available. Invariants checked in-module: area(merged) == area(outer) -
sum|area(holes)| and sum(triangle areas) == region area (1e-9*scale) — a
wrong-side bridge fails loudly instead of shipping slivers.

Orientation contract that made winding one rule: region loops are
"material on the left" (outers CCW, holes CW). For extrude the wall
normal = right-of-travel; for revolve the (u,v) axis basis is left-
handed (n x d = -1), so dP/dt x dP/dalpha is INWARD and the outward
wall triangulation is (a,c,b),(a,e,c) — pinned by signed-volume gates,
not eyeballed. Partial revolve of a profile on the -u side of the axis
folds to +u with the sweep origin rotated by pi (the wedge stays in
place; full sweeps are set-identical either way).

Render gate (`cargo test -j 3 -p physcad-view --test m5_capture`):
llvmpipe Cpu/Vulkan, 640x360, camera pitch 1.1 rad (steep — a 20-deg
elevation hides the through-bore inside the top-face band; see limit
note), measured px: plate blue 18933, bushing copper 5078, bore window
(background inside = see-through) 145 px anchored on the PROJECTED bore
center via the same view matrix; bushing tube-open probe 73 px.
docs/m5-extrude-csg.png sha256
8a3e01dfbd70083dcf6de610fe593ed0fd3b489a574488acd41f02d991950c5d
(640x360 RGBA, regenerated by the test; STL export path exercised by the
second m5_capture test on the same model).

License gate: NO new third-party deps in M5 — the loop extractor, bridge
ear-clipper and sweep builders are all in-house (license-clean by
construction); glam + manifold3d are pre-existing, already in LICENSES.md.

Known limits after M5:
- Revolve profile must lie STRICTLY on one side of the axis; profiles
  touching/crossing the axis (lathe solids) need degenerate-ring
  handling — deferred to the M6 timeline work, honest error until then.
- Vertices where >2 chains meet are rejected (no planar-face extraction
  yet); self-intersecting regions refused at triangulation.
- Extrude is straight +z only (no draft/partial twist); no rounded
  fillets between sketch and walls.
- Cap ear-clip is O(n^2)-ish; chord-tol 0.02 mm default keeps demo
  parts ~100-200 boundary points — fine; revisit for 10k-vertex blobs.
- No rotations in the CSG layer yet (move only) — carried from M3.

## M6 evidence (all commands run on this box, 2026-11-24)

Gates: `cargo fmt --check` clean; `cargo clippy -j 3 --all-targets` ->
0 warnings; `cargo test -j 3` -> 73 lib + 8 m3 + 11 m5 + 12 m6 + 10
sketch + 4 view lib + 6 capture/integration = 124 tests, 0 failed.

The DoD re-bind gate (`cargo test -j 3 -p physcad --test m6_gates
param_edit_rebinds_named_faces -- --nocapture`), verbatim before/after
face tables for `param plate_w = 60 -> 80` across
`plate@1/bore@2/part@3 = cut(plate, bore)`:

```
re-bind evidence (w=60):               re-bind evidence (w=80):
  F1:face:+x  c=( 30.000,0,0) a=480      F1:face:+x  c=( 40.000,0,0) a=480
  F1:face:-x  c=(-30.000,0,0) a=480      F1:face:-x  c=(-40.000,0,0) a=480
  F1:face:+y  c=(0, 6.000,0)  a=2321.59  F1:face:+y  c=(0, 6.000,0)  a=3121.59
  F1:face:-y  c=(0,-6.000,0)  a=2321.59  F1:face:-y  c=(0,-6.000,0)  a=3121.59
  F1:face:+z  c=(0,0, 20.000) a=720      F1:face:+z  c=(0,0, 20.000) a=960
  F1:face:-z  c=(0,0,-20.000) a=720      F1:face:-z  c=(0,0,-20.000) a=960
  F2:wall:cyl c=(0,0,0) tris=128 a=376.84  (identical: bore params untouched)
```

Same 7 keys, per-key triangle counts equal, centroids moved exactly
where the parameter says and nowhere else; volume delta = 20*12*40
within 1e-6. Rename `bore->hole` (keeping `@2`) and mid-tree insertion
(`knob@4`, `knob_at@5` between bore and `part@3`): face KEY SETS equal
and centroids BIT-identical (gated via f64::to_bits). Timeline API
(`Timeline::insert_at`, `Timeline::rename`) gates the same guarantees
without a file round trip.

Sketch-entity naming: extruded plate (4 lines + 1 circle) resolves
exactly `cap:bottom|top + wall:ab|bc|cd|da + wall:hole` (7 faces);
height edit 8->12 keeps all keys, `cap:top` centroid z 8->12, walls
stretch (tri counts fixed, areas grow). Full revolve of a rectangle
profile names 2 cylinders + 2 planes by profile entity; 90/120-deg
sweeps add `cap:start/cap:end`; the 180-deg sweep is PINNED as the
FM-3 contract: cap planes coincide, first record wins, both half-annuli
come back as `F1:cap:start#0/#1` — deterministic and visible, never
silently relabeled.

CLI readout on the example corpus (`./target/debug/physcad build …`),
zero `F0:unmatched`:
- examples/sketch-plate.pcm: 12 named faces — extrude family F1 (caps +
  5 entity walls; cap:top trimmed by the pocket: 2321.86 -> 2069.86
  = annulus − pocket footprint) + pocket walls F2:face:±x/±y/−z named
  by the TOOL box feature (cut generates no new names).
- examples/bore-plate.pcm: 8 named faces — 6 plate planes + F2:wall:cyl
  + F3:sphere surface (union/cut/move chain: sphere knob fused through
  box wall re-identifies across the oblique union seam, 4512 tris).

Failure modes FM-1..FM-7 (consumed names, split components, coincident
record-order ties, unbindable slivers, sliver false-positives, unpinned
id shifts, sketch-entity renames) documented in docs/NAMING.md; FM-3
and FM-1 pinned by executable tests. No B-rep kernel: the naming layer
is kernel-agnostic — it consumes only the welded f64 kernel mesh.

License gate: NO new third-party deps in M6 (timeline/naming are
in-house; LICENSES.md untouched).
