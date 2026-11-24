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

## Kernel-level deps (not used yet)

| Crate | Version | License | Planned |
|---|---|---|---|
| manifold3d | 0.4.2 | Apache-2.0 OR MIT | M3 CSG kernel (verified via crates.io API 2026-11-24) |
