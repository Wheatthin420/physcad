# LICENSES.md — third-party dependency record

Rule (physcad-mission): any third-party C/C++ or crate outside
MIT/Apache-2.0/BSD/ISC gets an entry here before first build. This file
opened at M2 when Cargo.toml gained real dependencies.

Scan method (2026-11-24, reproducible):
- Direct deps: `curl -s -H 'User-Agent: physcad-dev (davinci@localhost)'
  https://crates.io/api/v1/crates/<name>` (API requires a User-Agent header,
  otherwise it returns an empty body; the license is on `versions[].license`,
  not on the crate object).
- Full tree: `cargo metadata --format-version 1 --filter-platform
  x86_64-unknown-linux-gnu` → 178 external packages, license expression
  checked per package (commands and counts in docs/MILESTONES.md evidence).

## Direct dependencies

| Crate | Version | License | Used for |
|---|---|---|---|
| wgpu | 30.0.2 | MIT OR Apache-2.0 | GPU/offscreen rendering (M2 viewer, capture path) |
| egui | 0.37.0 | MIT OR Apache-2.0 | immediate-mode UI (control panel, canvas painting) |
| eframe | 0.37.0 | MIT OR Apache-2.0 | window shell + wgpu integration |
| glam | 0.34.3 | MIT OR Apache-2.0 | vector/matrix math (camera, meshes) |
| bytemuck | 1.25.2 | Zlib OR Apache-2.0 OR MIT | Pod casts for GPU buffers |
| image | 0.25.10 | MIT OR Apache-2.0 | PNG screenshot output (png feature only) |
| pollster | 1.0.3 | MIT OR Apache-2.0 | block_on for adapter/device setup (no async runtime yet) |
| manifold3d | 0.4.2 (default-features=false) | Apache-2.0 OR MIT | M3 CSG kernel (facade for manifold-csg 0.4.2 -> manifold-csg-sys 3.5.106). `parallel` feature OFF: single-threaded upstream manifold — deliberate for deterministic rebuilds and to skip the bundled TBB build. |

C++ actually compiled into the binary: manifold-csg-sys fetches
elalish/manifold **v3.5.5** (detached HEAD "Release v3.5.5 (#1947)" in its
fetch log) and cmake-builds it with g++. Its LICENSE at
`target/debug/build/manifold-csg-sys/*/out/manifold-src/LICENSE` verified
in-tree 2026-11-24: Apache-2.0 header, matching upstream repo. Build also
needs `cmake` (not on this box: installed standalone 3.31.6 to
~/.local/opt/cmake, PATH via ~/.bashrc).

Chosen license where dual: Apache-2.0 OR MIT (either).

## Transitive deps that need a note (all permissive, one option exercised)

| Crate | Version | Declared | Note |
|---|---|---|---|
| epaint_default_fonts | 0.37.0 | (MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0 | egui `default_fonts` feature. OFL is a font-file license: does NOT link-contaminate code, but if we ship binaries we must keep the font license texts. Consider custom font subset later. |
| self_cell | 1.3.0 | Apache-2.0 OR GPL-2.0-only | dual-licensed; we use the Apache-2.0 option (pulled in by winit). |
| unicode-ident, unicode-properties | 1.0.27 / 0.1.5 | (MIT OR Apache-2.0) AND Unicode-3.0 | Unicode data tables under the permissive Unicode-3.0 license. |
| memchr, termcolor, byteorder-lite | — | Unlicense OR MIT | MIT option. |
| rustix, linux-raw-sys | — | Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT | plain Apache-2.0 option. |
| zerocopy (+derive) | 0.8.65 | BSD-2-Clause OR Apache-2.0 OR MIT | Apache-2.0 option. |

Verdict: no GPL/LGPL/AGPL/MPL-only or non-permissive dependency exists in the
M2 tree. GPL exposure stays as documented in the survey (slvs at M4 — blocked
without user approval; OCCT at v2).

## M4 (2026-11-24): solver dependencies — none added

- slvs (SolveSpace core): GPL-3.0 (verified M1) — NOT vendored; own LM
  solver in crates/physcad/src/sketch/solver.rs.
- 3BCAD sketch solver (github.com/Three-Bugs-Studio/3bcad): repo Apache-2.0
  — studied for design (dual-number Jacobians, damping form, eigen rank);
  zero code linked.
- nalgebra (3BCAD's LA dep): crates.io API 2026-11-24 — Apache-2.0 since
  0.22, BSD-3-Clause before; earlier internal note calling it "GPL family"
  was WRONG, corrected here. Not needed: our dense Cholesky + Jacobi
  eigen + LM is ~200 LoC, dependency-free.

## Kernel-level deps (not used yet)

| Crate | Version | License | Planned |
|---|---|---|---|
| OCCT | — | LGPL-2.1 w/ exception | STEP import/export (v2) — decision deferred, see SURVEY.md |
