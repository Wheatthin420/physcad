# .pcm model file format (M3)

Plain text, one statement per line, `#` starts a comment. Units: mm.
Volumes mm^3; `density` in g/cm^3 (mass readout = density x volume/1000).

    # ---- parameters (f64 expressions, may reference earlier params) ----
    param plate_w = 60
    param plate_h = plate_w / 5        # 12
    param bore_r  = plate_h / 2 - 1    # 5

    density = 7.85                     # optional; default 1.0

    # ---- geometry nodes (root = LAST node defined) ----
    plate  = box(plate_w, plate_h, 40)
    bore   = cylinder(bore_r, plate_h + 4, 96)   # around +Y, centered
    knob   = sphere(6, 48, 96)                   # UV sphere r, rings, segs
    ring   = torus(12, 2, 128, 64)               # +Y axis: ring r, tube r, segs
    knob_h = move(knob, plate_w/4, plate_h/2, 0)
    stem   = union(plate, knob_h)
    part   = cut(stem, bore)                     # part is the root

Grammar: `box(w,h,d)`, `cylinder(r,h,segs)`, `sphere(r,rings,segs)`,
`torus(ring_r,tube_r,major_segs,minor_segs)`,
`move(node,x,y,z)`, `union(a,b)`, `cut(a,b)` (= a minus b), `intersect(a,b)`.
Node operands are bare names and must be defined above (declaration order =
dependency order; the file parses to a DAG of typed nodes, never re-parsed
at draw time). Segment counts must be whole numbers (cylinder segs >= 3,
sphere rings >= 2 segs >= 3, torus major/minor segs >= 3, cap 4_000_000).

Expressions: + - * / % ^ (right-assoc), unary +/-, parentheses, identifiers
(= params, plus built-ins pi tau e), functions:
abs sqrt cbrt ln log10 exp floor ceil sin cos tan asin acos atan (1 arg),
min max pow atan2 hypot mod (2 args). Domain errors (sqrt(-1), 1/0,
unknown name) report file:line:col.

Build + readout:

    cargo build -j 3 -p physcad --bin physcad
    ./target/debug/physcad build examples/bore-plate.pcm --stl part.stl

Determinism contract: same file + same binary => bit-identical volume,
mesh and STL bytes (single-threaded manifold, fixed STL header, no
timestamps). Verified in tests/m3_gates.rs and cross-process in
MILESTONES.md.

## M4 sketch statements

Same line-based grammar; sketch entities/constraints are statements of
their own (they do not enter the CSG graph; the solid root stays the
last *geometry node*):

    sk = newsketch()                 # opens a sketch; names live in the file namespace
    p  = point(sk, X, Y)             # 2 DOF each
    l  = line(sk, P1, P2)            # endpoints are point entities (shared structure, not params)
    c  = circle(sk, CENTER, R)       # center point entity + 1 own param (radius)
    a  = arc(sk, CENTER, START, END) # three point entities; CCW sweep start->end
    co = coincident(P1, P2)          # 2 eqs
    d  = distance(P1, P2, EXPR)      # mm
    an = angle(L1, L2, DEG)          # signed CCW from dir(L1) to dir(L2), degrees
    h  = horizontal(L) | v = vertical(L)
    r  = radius(C, EXPR)             # circle or arc (arc radius = |start-center|)
    t  = tangent(L, C) | tangent(L, ARC) | tangent(C1, C2)  # cc = external
    f  = fix(P, X, Y)                # anchor

Operand names must be declared above; all operands of one constraint
must belong to one sketch. Values are f64 expressions over earlier
`param`s. Solve report per sketch: eqs, params, rank, dof, max|residual|,
named redundant constraints — printed by `physcad build` (a sketch that
fails to converge is a warning, not a build error, until features need it).

Example: examples/sketch-bracket.pcm (rectangle + fillet circle tangent
to two edges + quarter arc; 17 eqs / 17 params -> dof 0, max|res| 5.7e-14).

## M5: extrude / revolve — sketch regions become solids

Sketch-backed geometry NODES (they enter the CSG graph, compose with
box/cylinder/… and each other; each closes the solved sketch's curves
into regions and builds a closed manifold mesh itself, which the kernel
then consumes like any primitive):

    slab = extrude(SKETCH, H)                 # solid z in [0, H]
    slab = extrude(SKETCH, H, OFF)            # solid z in [OFF, OFF+H]
    slab = extrude(SKETCH, H, OFF, SEGS)      # SEGS-gon arc sampling
    bush = revolve(SKETCH, AXIS_LINE)         # 360 deg about an in-plane line entity
    bush = revolve(SKETCH, AXIS_LINE, DEG)    # 0 < DEG <= 360
    bush = revolve(SKETCH, AX, DEG, SWEEP_SEGS[, PROFILE_SEGS])

    # bore plate straight from a solved sketch, then CSG-composed:
    part = cut(extrude(plate, 12, 0, 96), move(box(18, 14, 4), 18, 11, 10))

Semantics:
- The sketch is LM-SOLVED first (M4), then curves are welded (1e-9 mm)
  into closed loops: outer boundaries CCW, holes CW; island-in-hole
  nesting supported (every even-depth loop owns a region).
- Caps are triangulated by our own bridge+ear-clipper; walls from loop
  edges. The generated f64 mesh MUST pass closed_manifold_check (edge
  pairing + Euler) before the kernel sees it; evaluate() re-checks the
  display mesh. Multiple regions in one sketch union into one node.
- SEGS / SWEEP_SEGS: exact control of polygonization (the gates rely on
  it — the generated mesh is then the exact polyhedron). profile_segs
  omitted = chord-sagitta 0.02 mm sampling.
- Limits (honest errors, see MILESTONES "Known limits"): >2 chains at
  one vertex rejected; self-intersecting regions rejected at
  triangulation; revolve profile must lie strictly on ONE side of the
  axis (touching/crossing deferred to the timeline milestone).

Example: examples/sketch-plate.pcm (solved rectangle+bore sketch ->
extrude -> corner-pocket cut; V exact-polyhedral within 1.8e-11).
