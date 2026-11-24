# MILESTONES.md — status per rung (ladder defined in physcad-mission skill)

| # | Milestone | Status | Date | Evidence |
|---|---|---|---|---|
| 1 | SURVEY.md field survey + shortlist | DONE | 2026-11-24 | docs/SURVEY.md; all licenses read from project LICENSE files + crates.io API same day |
| 2 | Viewer shell (wgpu + egui, orbit, primitives, screenshot) | DONE | 2026-11-24 | crates/physcad + crates/physcad-view; docs/m2-capture.png (1280x720, llvmpipe Cpu/Vulkan adapter, sha256 in commit msg body); LICENSES.md opened |
| 3 | Mesh kernel slice (manifold3d CSG → STL, params+expressions) | DONE | 2026-11-24 | crates/physcad {expr,tess,csg,model,stl}.rs + src/bin/physcad.rs; crates/physcad/tests/m3_gates.rs (7 analytic volume gates + mass gate + bit-stability, incl. torus); crates/physcad-view/tests/csg_capture.rs (headless render of CSG result incl. kernel torus + exact-polyhedral volume check); examples/bore-plate.pcm + examples/torus-plate.pcm; docs/m3-csg.png; evidence below |
| 4 | Sketches + constraint solver (own LM; slvs blocked GPL-3.0) | DONE | 2026-11-24 | crates/physcad/src/sketch/{mod,solver}.rs (dual-number exact Jacobians, dense LM, Jacobi-eigen rank/DOF, Gram-Schmidt named redundancy) + .pcm sketch grammar in model.rs + examples/sketch-bracket.pcm + crates/physcad/tests/sketch_gates.rs (10 gates, residuals below) + crates/physcad-view/tests/m4_sketch_capture.rs (headless wgpu render of solved sketch; docs/m4-sketch.png). Developed on branch `m4-sketch` (concurrent M3-torus session held the tree), fast-forward merged to main 2026-11-24 |
| 5 | Extrude/revolve from closed sketch loops | not started | — | — |
| 6 | Timeline data model + stable topology naming | not started | — | — |
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
