# NAMING.md — stable face/edge naming across feature recomputation (M6)

Mission context: docs/SURVEY.md §2 names topological naming as the core
risk; the mandate there is "face identity must be carried as explicit
creation-history data in the feature tree, never re-derived from
traversal order". This file records the algorithm we shipped, why it is
deterministic, and — as the milestone demands — its known failure modes.

## Decision: mesh-based continuity heuristic, no B-rep kernel

We do NOT run a B-rep kernel and we do NOT trust manifold to keep any
provenance across booleans (it promises none). Faces of the evaluated
solid are recovered by matching rebuilt mesh triangles against the
*declared* analytic surfaces of the feature timeline. A B-rep migration
(monstertruck, Apache-2.0) stays a documented decision point (MILESTONES
M5 note); this milestone deliberately proves the data model first,
because naming-by-creation-history works for any kernel that can put a
triangle on a face.

## The identity data model (timeline.rs)

- Every feature has a `FeatureId` (nonzero, monotonic, never reused).
  `GeoNode` operands (`move/union/cut/intersect`) reference features by
  `FeatureId`, never by vector position. `.pcm` statements can pin an
  id: `plate@1 = box(...)`; unpinned features take `next_id` at parse
  time. `Timeline::insert_at` / `rename` are the mid-tree edit
  primitives; both are identity-preserving by construction.
- A face key is `F{feature_id}:{structural tag}`. Tags come from the
  feature's construction semantics:
  - box: `face:±x/±y/±z` · cylinder: `wall:cyl`, `cap:±y` ·
    sphere/torus: `surface`
  - extrude: `cap:bottom`, `cap:top`, `wall:<sketch-entity-name>`
  - revolve: `cap:start`, `cap:end` (partial sweeps),
    `wall:<sketch-entity-name>`
  - CSG nodes mint no names: a cut's new wall faces lie on the TOOL's
    surfaces and inherit the tool feature's key (the bore wall belongs
    to the bore feature).
- When one logical face resolves to several mesh-connected components,
  keys get deterministic `#k` suffixes (components sorted by
  total-ordered centroid).

## The algorithm (naming.rs, run inside csg::evaluate_with)

1. Each leaf feature publishes a `FaceTable`: one `FaceRecord` per face
   = (FeatureId, tag, analytic `Surface` ∈ plane/cylinder/sphere/cone/
   torus). Surfaces are derived from the feature's typed parameters —
   for extrude/revolve walls, from the SOLVED sketch entity parameters
   (a wall named `wall:hole` lives on the exact circle cylinder,
   independent of how many segments sampled it).
2. `Move` translates the operand's surfaces. `union/cut/intersect`
   merge operand tables in order A-then-B; the merged table is the
   declaring set for the result.
3. After the kernel boolean, EVERY output triangle is classified: all
   three vertices must lie within `MATCH_EPS = 1e-7 mm` of the
   declaring surface; the first table record that matches wins (A-first
   order makes ties deterministic).
4. Triangles group by (record, mesh-connected component) via
   union-find on welded kernel vertices; each group emits a `NamedFace`
   (key, triangle count, f64 area, centroid). Unmatched triangles never
   vanish and never borrow a neighbor's name: they collect under the
   reserved key `F0:unmatched`, which CLI output warns about and the
   gates assert is empty on the model corpus.

Re-bind contract (tested in tests/m6_gates.rs): edit an earlier
parameter / rename a mid-tree feature / insert a mid-tree feature ->
rebuild -> the named-face key set is unchanged, and each face's
geometry moves exactly where the parameter says (e.g. `F1:face:+x`
centroid 30 -> 40 when `plate_w` goes 60 -> 80, while `F2:wall:cyl`
stays put down to its triangle count).

## Known failure modes (explicit; three are pinned by tests)

- **FM-1 consumed faces disappear.** If a later feature eats a face
  entirely (e.g. a cut removing the cylinder's end cap), that key is
  simply absent from the root table of faces. Any downstream reference
  to it must fail loudly — we never guess a successor. (Tested:
  `F2:cap:±y` absent from the bore-plate root, `F1:cap:*` present.)
- **FM-2 face splits change component indices.** A face whose surface
  survives but whose mesh region splits into k components changes
  `key` -> `key#0..#k-1`. The base identity (feature+tag) survives;
  component enumeration is total-ordered so it is at least
  deterministic. Consumers needing split-stable references need a
  component-content signature — deferred to the editor milestone.
- **FM-3 coincident declaring surfaces tie-break by record order.**
  Two features declaring the same plane/surface (flush union faces,
  adjacent collinear sketch edges, a construction line lying exactly
  on a wall, cap planes coinciding at a 180-degree revolve) resolve to
  the FIRST record in table order (operand A, then B; entity file
  order). Deterministic — but the "winner" may not be the name a human
  intended. Pinned executable:
  `half_turn_cap_coincidence_is_deterministic_and_visible`.
- **FM-4 unbindable slivers.** Near-tangential boolean intersections can
  create triangles whose three vertices land on NO declaring surface
  within 1e-7 mm. They collect as `F0:unmatched` (visible in CLI
  warning, fails the gates) — the heuristic reports its own misses
  rather than mislabeling. No corpus model currently triggers it.
- **FM-5 curved-surface false-positive radius.** A triangle can sit on
  two declaring surfaces at once only if it is sliver-small (a plane is
  tangent to a cylinder along a line, not an area). Table order decides;
  area-based scoring was considered and rejected as non-deterministic
  near-degenerate.
- **FM-6 unpinned ids are positional.** `NAME@K` pinning is what makes
  an inserted feature safe. If an author (or a bad editor write) lets
  ids auto-number and then inserts a feature EARLIER in the file, every
  later feature's id shifts and its face keys rename. The parser keeps
  appends safe (ids never shift backwards); the GUI timeline must always
  pin ids at feature creation. Tests pin ids for exactly this reason.
- **FM-7 sketch entity names are the wall identity.** `wall:<entity>`
  embeds the sketch ENTITY name: renaming a sketch entity renames its
  wall. Feature renames are free (ids), entity renames are not yet —
  entity ids are the direct follow-up (M7 input if CAM ops bind edges).
- **Out of scope (honest):** edge naming (faces only this milestone),
  vertex naming, assembly-level names (feature ids are per-file), and
  fillet/chamfer faces (no such features exist yet — when they do, they
  publish their own analytic surfaces: cone-sphere blend families).

## What is NOT heuristic

- Identity storage: ids and tags are typed data in the timeline.
- Determinism: table order (A-then-B, file order), first-match wins,
  total-ordered component indices. Same text + same binary => the face
  map is bit-identical across rebuilds (gated).
- The geometric test itself: closed-form surface distance in f64,
  tolerance 5 orders of magnitude above kernel noise and 8 below any
  feature dimension.
