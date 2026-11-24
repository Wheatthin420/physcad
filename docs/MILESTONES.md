# MILESTONES.md — status per rung (ladder defined in physcad-mission skill)

| # | Milestone | Status | Date | Evidence |
|---|---|---|---|---|
| 1 | SURVEY.md field survey + shortlist | DONE | 2026-11-24 | docs/SURVEY.md; all licenses read from project LICENSE files + crates.io API same day |
| 2 | Viewer shell (wgpu + egui, orbit, primitives, screenshot) | not started | — | — |
| 3 | Mesh kernel slice (manifold3d CSG → STL, params+expressions) | not started | — | — |
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
