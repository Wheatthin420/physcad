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
    knob_h = move(knob, plate_w/4, plate_h/2, 0)
    stem   = union(plate, knob_h)
    part   = cut(stem, bore)                     # part is the root

Grammar: `box(w,h,d)`, `cylinder(r,h,segs)`, `sphere(r,rings,segs)`,
`move(node,x,y,z)`, `union(a,b)`, `cut(a,b)` (= a minus b), `intersect(a,b)`.
Node operands are bare names and must be defined above (declaration order =
dependency order; the file parses to a DAG of typed nodes, never re-parsed
at draw time). Segment counts must be whole numbers (cylinder segs >= 3,
sphere rings >= 2 segs >= 3, cap 4_000_000).

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
