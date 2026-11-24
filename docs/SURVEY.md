# SURVEY.md — Mission Milestone 1: field survey

Date of verification: 2026-11-24. Every license claim below was read today from the
project's own LICENSE/COPYING file (via raw.githubusercontent) or the crates.io API;
maturity claims (release tag + date, last push, stars) from the GitHub REST API.
Exact endpoints are listed in §4 so any row can be re-derived.

## 1. Projects

| Project | What it is | License (source, checked 2026-11-24) | Maturity (checked 2026-11-24) | Borrow | License-clean alternative |
|---|---|---|---|---|---|
| PhysCAD | Closed-source commercial CAD+CAM desktop app; the mission's target spec. $49.99 one-time, 3 machines, win/mac/linux, "no subscription, no phoning home". | Proprietary (commercial product; https://physcad.io/buy pricing page) | v0.9.19 shown on https://physcad.io (beta-grade) | The whole product spec: timeline + direct push/pull, stable face/edge naming, one-click drawings, CAM sim with gouge/rapid/flute/vise checks, auto vise/stock/WCS, per-key offline use | n/a — spec to build to, not code to reuse |
| SolveSpace | 2D/3D parametric CAD: constrained sketches + extrude/lathe/boolean, minimal B-rep. https://github.com/solvespace/solvespace | GPL-3.0 (COPYING.txt header "Version 3": https://raw.githubusercontent.com/solvespace/solvespace/master/COPYING.txt) | v3.3 released 2026-10-21; pushed 2026-11-23; 4,268 stars, 91+ contributors (repo page) | Constraint taxonomy (coincident, distance, angle, tangent, symmetry) and the request→solve UX; its file format as a sketch interchange target | Self-written Levenberg–Marquardt solver (M4 fallback); 3BCAD's LM sketch solver as an Apache-2.0 prior art: https://github.com/Three-Bugs-Studio/3bcad |
| slvs | SolveSpace's constraint solver extracted as a standalone C library (`lib/` in the SolveSpace repo); Rust binding crate `slvs` v0.6.0 | GPL-3.0 — binding crate states GPL-3.0 (https://crates.io/api/v1/crates/slvs, repo https://github.com/thekakkun/rust_slvs); core inherits SolveSpace's GPL-3.0 | Crate last published 2023-06-20 (crates.io API); core solver code active in upstream repo | The C API shape itself: point/line/arc/curve entities, constraint groups, `solve()` over a param block — good API to clone in clean-room Rust | Same as SolveSpace row: own LM solver. GPL-3.0 dependency needs explicit user approval before vendoring (mission rule) |
| FreeCAD | Full parametric 3D CAD app on the OCCT kernel; scripting + workbenches. https://github.com/FreeCAD/FreeCAD | LGPL-2.1 (LICENSE file header via https://raw.githubusercontent.com/FreeCAD/FreeCAD/main/LICENSE; GitHub API confirms LGPL-2.1; note: selected add-on workbenches carry other licenses) | 1.1.4 released 2026-09-28; pushed 2026-11-24; 35,758 stars | Documented failure mode: the topological naming problem — see §2. Also the Path/CAM workbench op taxonomy and post-processor-per-controller pattern | None needed — study only, no code reuse (LGPL is linkable but FreeCAD is an app, not a library) |
| OpenSCAD | Scripting solid modeler: declarative CSG language, customizer, STL/3MF out. https://github.com/openscad/openscad | GPL-2.0 + CGAL linking exception (COPYING file: "Version 2, June 1991" + special CGAL exception header: https://raw.githubusercontent.com/openscad/openscad/master/COPYING) | Tagged release 2021.01 (2021-02-07) but development is continuous: pushed 2026-11-24, 10,734 stars, 274+ contributors (repo page) | Declarative CSG tree as a saved-file model; parameter/customizer concept | Manifold (Apache-2.0) for the geometry — it is the drop-in replacement backend OpenSCAD itself adopted; own Rust CSG tree over meshes for M3 |
| CadQuery | Python parametric CAD scripting framework on OCCT; fluent selector/workplane API. https://github.com/CadQuery/cadquery | Apache-2.0 (LICENSE file: "Apache Public License, v 2.0": https://raw.githubusercontent.com/CadQuery/cadquery/master/LICENSE) | v2.8.0 released 2026-06-20; pushed 2026-11-23; 6,166 stars, 100+ contributors | Selector grammar (`faces('>Z')`, `edges('|X')`) and chained-workplane modeling ergonomics — reimplement in Rust over our kernel | n/a — Apache-2.0, so code may be lifted with attribution if ever useful (its kernel is OCCT underneath, which is the LGPL part) |
| OpenCascade (OCCT) | C++ 3D modeling platform: B-rep kernel, STEP/IGES exchange, visualization. https://github.com/Open-Cascade-SAS/OCCT | LGPL-2.1 **with additional exception**; commercial license alternative (docs: https://old.opencascade.com/doc/occt-7.2.0/overview/html/index.html; LICENSE_LGPL_21.txt in repo) | V8_0_2 released 2026-11-22; pushed 2026-11-23; 3,206 stars; the industry workhorse behind FreeCAD and CadQuery | Defer: STEP import/export is the reason to ever link it (v2 decision). Study TopoDS shape model before designing our B-rep migration | truck/monstertruck (Apache-2.0) for Rust B-rep; Manifold for mesh work; keep OCCT out of the build until STEP is actually needed |
| Truck | Rust B-rep kernel with NURBS: geometry, topology, boolean shapeops, meshing, STEP I/O crate. https://github.com/ricosjp/truck | Apache-2.0 (repo LICENSE; crates.io `truck-geometry` 0.5.0 Apache-2.0: https://crates.io/api/v1/crates/truck-geometry) | Upstream crates stagnant: truck-geometry 0.5.0 last published 2024-09-20 (222,638 dl); maintained fork `monstertruck` 0.4.3 published 2026-11-23 (https://crates.io/api/v1/crates/monstertruck, repo https://github.com/virtualritz/monstertruck, Apache-2.0); 1,703 stars | Primary Rust B-rep candidate for the eventual kernel migration (M5+ decision point). 3BCAD proves the full stack truck+LM solver+wgpu+egui works: https://github.com/Three-Bugs-Studio/3bcad (Apache-2.0) | — (it IS the clean alternative to OCCT) |
| Manifold | Mesh geometry library guaranteeing manifold output: robust boolean CSG, trimesh↔manifold, corefinement. https://github.com/elalish/manifold | Apache-2.0 (repo LICENSE; Rust crate `manifold3d` 0.4.2 = "Apache-2.0 OR MIT": https://crates.io/api/v1/crates/manifold3d) | v3.5.5 released 2026-11-13; pushed 2026-11-24 (commit hours old); 2,439 stars, 96+ contributors | M3 kernel: CSG over triangle meshes with a manifoldness guarantee — the mission's chosen antidote to hand-rolled boolean robustness | — (already license-clean; the `csg` crate fallback named in the mission ladder does NOT exist on crates.io — 404 from API 2026-11-24 — so Manifold is the only path) |
| OpenCAMLib | C++ 3-axis toolpath library: drop-cutter (2D/3D offset), push-cutter/waterline, rest-milling; cutters flat/ball/bull/cone/composite; Python/JS bindings. https://github.com/aewallin/opencamlib | LGPL-2.1 (GitHub API license field + repo README: "From August 2018 OpenCAMLib is released under LGPL license") | Release 2023.01.11 (2023-01-11); last push 2026-10-17; 555 stars — aging, small team | Algorithm concepts only: the drop-cutter vs waterline split, composite-cutter representation, zig-zag/raster planning | Rust reimplementation over kernel meshes (M7); `cavalier_contours` crate (MIT OR Apache-2.0, v0.9.0 2026-08-20, 155,986 dl: https://crates.io/api/v1/crates/cavalier_contours) for arc-preserving 2D offsets |

## 2. Flagged risk: topological naming (verified references)

- Definition and impact (faces/edges change internal names after pad/cut/fillet,
  breaking dependent features): https://wiki.freecad.org/Topological_naming_problem
- "Fix" (realthunder's algorithm) reduces but does not solve it — wiki says widely
  mis-reported as solved: same URL.
- FreeCAD's GSoC naming project (creation-history identifier idea):
  https://wiki.freecad.org/Topological_Naming_Project
- Group tracker issue since 2012, still open: https://tracker.freecad.org/view.php?id=922
- Consequence for PhysCAD clone: face identity must be carried as explicit
  creation-history data in the feature tree (M6), never re-derived from traversal
  order. The 3BCAD project states "rebuild with persistent face naming" as a core
  crate feature (https://github.com/Three-Bugs-Studio/3bcad) — closest open prior
  art to study for M6.

## 3. Shortlist — kernel / solver / viewport

| Slot | Pick | License | Backup | License | Rejected | Reason for rejection |
|---|---|---|---|---|---|---|
| Geometry kernel (M3: mesh CSG) | Manifold via `manifold3d` 0.4.2 | Apache-2.0 OR MIT (crate) / Apache-2.0 (upstream) | truck/`monstertruck` for B-rep migration (M5+ decision) | Apache-2.0 both | OCCT | LGPL-2.1+exception; keep out until STEP needed (v2); OpenSCAD's CGAL path: GPL-2 |
| Sketch constraint solver (M4) | Own Levenberg–Marquardt in Rust | clean (ours) | — | — | slvs / libslvs | GPL-3.0 (https://crates.io/api/v1/crates/slvs) — hard block without explicit user approval |
| 2D path offsets (M7 CAM) | `cavalier_contours` 0.9.0 | MIT OR Apache-2.0 | own code | clean | OpenCAMLib as dependency | LGPL-2.1; concepts only, no link |
| Viewport / renderer (M2) | `wgpu` 30.0.2 | MIT OR Apache-2.0 (https://crates.io/api/v1/crates/wgpu) | — | — | OCCT visualization | ties to OCCT (LGPL) |
| UI toolkit (M2) | `egui` + `eframe` 0.37.0 | MIT OR Apache-2.0 (https://crates.io/api/v1/crates/egui) | — | — | Qt (FreeCAD/OpenSCAD route) | GPL/commercial fork cost; egui proven by 3BCAD for exactly this app class |

Net plan: M2–M3 stack (wgpu + eframe + manifold3d) and M4 solver are 100%
MIT/Apache — no GPL/LGPL anywhere in the first four milestones. First license
decision gate is M5+ (B-rep migration: monstertruck Apache-2.0 vs deferred OCCT
LGPL for STEP). LICENSES.md must be opened when the first third-party dependency
enters `Cargo.toml` (nothing has yet as of this milestone — docs only).

## 4. Provenance — commands that produced this data (2026-11-24)

- GitHub repo facts (stars, pushed_at, license spdx, latest release):
  `curl -s https://api.github.com/repos/<owner>/<repo>` and
  `.../releases/latest` for: solvespace/solvespace, FreeCAD/FreeCAD,
  openscad/openscad, CadQuery/cadquery, elalish/manifold, ricosjp/truck,
  aewallin/opencamlib, Open-Cascade-SAS/OCCT.
- License text headers: `curl -sL https://raw.githubusercontent.com/<repo>/<branch>/LICENSE|COPYING|LICENSE_LGPL_21.txt | head`
  (solvespace path is COPYING.txt; OCCT path is LICENSE_LGPL_21.txt; GitHub API
  license field returned NOASSERTION for OpenSCAD and CadQuery, so the file
  headers are the authority used above).
- Crate facts (version, license, download count, publish date):
  `curl -s https://crates.io/api/v1/crates/<name>` for slvs, manifold3d,
  truck-geometry, monstertruck, wgpu, egui, eframe, occt-sys,
  cavalier_contours, csg (404 — does not exist).
- OCCT license statement incl. "additional exception":
  https://old.opencascade.com/doc/occt-7.2.0/overview/html/index.html
