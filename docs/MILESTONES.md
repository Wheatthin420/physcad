# MILESTONES.md — status per rung (ladder defined in physcad-mission skill)

| # | Milestone | Status | Date | Evidence |
|---|---|---|---|---|
| 1 | SURVEY.md field survey + shortlist | DONE | 2026-11-24 | docs/SURVEY.md; all licenses read from project LICENSE files + crates.io API same day |
| 2 | Viewer shell (wgpu + egui, orbit, primitives, screenshot) | DONE | 2026-11-24 | crates/physcad + crates/physcad-view; docs/m2-capture.png (1280x720, llvmpipe Cpu/Vulkan adapter, sha256 in commit msg body); LICENSES.md opened |
| 3 | Mesh kernel slice (manifold3d CSG → STL, params+expressions) | DONE | 2026-11-24 | crates/physcad {expr,tess,csg,model,stl}.rs + src/bin/physcad.rs; crates/physcad/tests/m3_gates.rs (5 analytic volume gates + bit-stability); crates/physcad-view/tests/csg_capture.rs (headless render of CSG result + exact-polyhedral volume check); examples/bore-plate.pcm; docs/m3-csg.png; evidence below |
| 4 | Sketches + constraint solver (own LM; slvs blocked GPL-3.0) | not started | — | — |
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
`cargo test -j 3` -> 39 lib + 6 gate + 2 view lib + 3 capture/integration tests, 0 failed.

Analytic volume gates (`cargo test -j 3 -p physcad --test m3_gates -- --nocapture`),
tolerance 1e-6 mm^3 absolute, kernel volume = manifold f64:

```
GATE union box+cyl:        got 4506.283185153  analytic 4506.283185307  |err| 1.540e-7
GATE cut cyl bore:         got  193.716814847  analytic  193.716814693  |err| 1.540e-7
GATE intersect box∩cyl:    got    9.424777730  analytic    9.424777961  |err| 2.310e-7
GATE union box+hemisphere: got    1.032724641  analytic    1.032724923  |err| 2.824e-7
GATE cut spherical cavity: got    0.934550718  analytic    0.934550153  |err| 5.647e-7
```

Math: cylinders = inscribed 16384-gon prisms (error budget V·(2π/n)²/6 ≈
1.5e-7 — measured matches); spheres = UV-polyhedra r=1/4 at 1024×1024
(measured ≈2.3× the naive (π²/m²+4π²/3n²)/6 estimate, still 1.8× margin);
boxes exact. Demo-part cross-check (csg_capture test) matches its exact
polyhedral closed form (prismoid sum) at dev 0.000e0.

Bit-stability:
- In-process: `scripted_rebuild_is_bit_stable` — two evaluates of the same
  parsed model -> volume bit-identical, mesh equal, 410384 STL bytes equal.
- Cross-process: two `./target/debug/physcad build examples/bore-plate.pcm --stl …`
  runs -> both `sha256 16004736ee145b81aa7a6c3590dc855673304fd62c447d4e574a376202068ace`,
  CLI readout identical (V=28309.777414 mm^3, mass 222.2318 g @7.85 g/cm^3,
  genus 1, 5010 tris).
- STL f32 quantization, f64-summed: relative volume drift 5.3e-10.

Kernel setup: manifold3d 0.4.2 w/o default features (no TBB; single-threaded
upstream manifold v3.5.5, LICENSE verified in build tree, see LICENSES.md).
cmake 3.31.6 standalone at ~/.local/opt/cmake (no apt on box, PEP 668
blocked pip) — added to PATH via ~/.bashrc.

Genus convention (verified, tests encode it): through-hole -> +1; extra
closed shell (disconnected component OR interior cavity) -> -1.

Known limits after M3: no rotations yet (move only), UV-sphere poles pinch
triangle sizes, no units system, ASCII STL not implemented.
