//! M6 gates: the feature timeline re-binds named faces deterministically
//! across mid-tree edits (docs/NAMING.md algorithm; failure modes there).
//!
//! Each gate rebuilds a model from `.pcm` TEXT variants (param edit,
//! feature rename, mid-file feature insertion), evaluates, and asserts
//! named-face identity — the milestone's definition of done. Every gate
//! also asserts ZERO `F0:unmatched` groups: on these models the naming
//! heuristic must bind every kernel triangle.

use std::collections::BTreeMap;

use physcad::Solid;
use physcad::csg;
use physcad::model::parse_model;
use physcad::timeline::FeatureId;

/// key -> (num_tris, centroid, area_mm2)
fn face_map(s: &Solid) -> BTreeMap<String, (usize, [f64; 3], f64)> {
    s.faces
        .iter()
        .map(|f| (f.key.clone(), (f.num_tris, f.centroid, f.area_mm2)))
        .collect()
}

fn keys(s: &Solid) -> Vec<String> {
    s.faces.iter().map(|f| f.key.clone()).collect()
}

fn assert_all_named(s: &Solid) {
    let unmatched: Vec<&String> = s
        .faces
        .iter()
        .filter(|f| !f.feature.is_real())
        .map(|f| &f.key)
        .collect();
    assert!(
        unmatched.is_empty(),
        "unmatched face groups {unmatched:?}; keys were {:?}",
        keys(s)
    );
}

fn centroid_close(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    a.iter().zip(b.iter()).all(|(x, y)| (x - y).abs() <= tol)
}

// ---------------------------------------------------------------------------
// Gate 1: parameter edit — downstream named faces re-bind, geometry moves.
// ---------------------------------------------------------------------------

const PLATE_V1: &str = "\
param plate_w = 60
plate@1 = box(plate_w, 12, 40)
bore@2  = cylinder(5, 16, 64)
part@3  = cut(plate, bore)
";

const PLATE_V2: &str = "\
param plate_w = 80
plate@1 = box(plate_w, 12, 40)
bore@2  = cylinder(5, 16, 64)
part@3  = cut(plate, bore)
";

fn build(src: &str) -> Solid {
    let m = parse_model(src, "m6-gate.pcm").expect("parses");
    csg::evaluate_with(&m.graph, &m.sketches).expect("evaluates")
}

#[test]
fn param_edit_rebinds_named_faces() {
    let v1 = build(PLATE_V1);
    let v2 = build(PLATE_V2);
    assert_all_named(&v1);
    assert_all_named(&v2);

    // The named-face SET is identical across the rebuild — that is the
    // re-bind contract; identity did not move with triangle order.
    assert_eq!(keys(&v1), keys(&v2), "face keys must be invariant");
    assert_eq!(
        v1.faces.len(),
        7,
        "plate 6 faces + bore wall: {:?}",
        keys(&v1)
    );

    // Geometry re-binds where the parameter says, and ONLY there.
    let x1 = v1.face("F1:face:+x").unwrap();
    let x2 = v2.face("F1:face:+x").unwrap();
    assert!(centroid_close(x1.centroid, [30.0, 0.0, 0.0], 1e-9));
    assert!(
        centroid_close(x2.centroid, [40.0, 0.0, 0.0], 1e-9),
        "w 60->80 must move the +x face centroid 30->40"
    );
    assert_eq!(x1.num_tris, x2.num_tris);

    // The bore feature was untouched: its wall re-binds to the same place.
    let w1 = v1.face("F2:wall:cyl").unwrap();
    let w2 = v2.face("F2:wall:cyl").unwrap();
    assert!(centroid_close(w1.centroid, w2.centroid, 1e-9));
    assert_eq!(
        w1.num_tris, w2.num_tris,
        "tessellation of the bore unchanged"
    );
    assert!(
        (w1.area_mm2 - 2.0 * std::f64::consts::PI * 5.0 * 12.0).abs() < 2.0,
        "bore wall area ~ 2*pi*r*h, got {}",
        w1.area_mm2
    );

    // Untouched faces bit-stable (same param for them): -z centroid.
    let z1 = v1.face("F1:face:-z").unwrap();
    let z2 = v2.face("F1:face:-z").unwrap();
    assert!(centroid_close(z1.centroid, z2.centroid, 1e-9));
    // Volume moved by exactly the plate delta (60->80) * 12 * 40 minus 0.
    let dv = v2.volume_mm3 - v1.volume_mm3;
    assert!((dv - 20.0 * 12.0 * 40.0).abs() < 1e-6, "volume delta {dv}");

    // Evidence for MILESTONES: before/after named-face bindings.
    for (tag, s) in [("w=60", &v1), ("w=80", &v2)] {
        eprintln!("re-bind evidence ({tag}):");
        for f in &s.faces {
            eprintln!(
                "  {:<16} tris={:<4} area={:>9.4}  centroid=({:>8.3}, {:>7.3}, {:>7.3})",
                f.key, f.num_tris, f.area_mm2, f.centroid[0], f.centroid[1], f.centroid[2]
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Gate 2: rename a mid-tree feature — identity (FeatureId-keyed faces)
// must survive even though the file name changed.
// ---------------------------------------------------------------------------

const PLATE_RENAMED: &str = "\
param plate_w = 60
plate@1 = box(plate_w, 12, 40)
hole@2  = cylinder(5, 16, 64)
part@3  = cut(plate, hole)
";

#[test]
fn midtree_rename_preserves_named_face_identity() {
    let v1 = build(PLATE_V1);
    let v2 = build(PLATE_RENAMED);
    assert_all_named(&v2);

    // Same keys: face names embed feature IDS, never user names.
    assert_eq!(keys(&v1), keys(&v2), "bore->hole rename changed face keys");
    assert!(v2.face("F2:wall:cyl").is_some(), "id F2 identity survived");
    assert!(v2.face("F99:wall:cyl").is_none());

    // Geometry was untouched -> centroids bit-identical across rename.
    for (k, (_, c1, _)) in face_map(&v1) {
        let (_, c2, _) = *face_map(&v2).get(&k).unwrap();
        let c1b: [u64; 3] = c1.map(f64::to_bits);
        let c2b: [u64; 3] = c2.map(f64::to_bits);
        assert_eq!(c1b, c2b, "rename perturbed {k} geometry");
    }
}

// The same edit through the TIMELINE API (not the file): rename by id.
#[test]
fn timeline_api_rename_rebinds_without_key_change() {
    let mut m = parse_model(PLATE_V1, "m6-gate.pcm").unwrap();
    let before = keys(&build(PLATE_V1));
    m.graph
        .rename(FeatureId(2), "hole")
        .expect("rename must succeed");
    // Operands still resolve by id: the cut still references F2.
    let s = csg::evaluate(&m.graph).unwrap();
    assert_all_named(&s);
    assert_eq!(keys(&s), before);
}

// ---------------------------------------------------------------------------
// Gate 3: insert a feature mid-timeline — downstream references and named
// faces must be untouched (the ladder's explicit DoD).
// ---------------------------------------------------------------------------

const PLATE_INSERTED: &str = "\
param plate_w = 60
plate@1 = box(plate_w, 12, 40)
bore@2  = cylinder(5, 16, 64)
knob@4  = box(4, 4, 4)
knob_at@5 = move(knob, 20, 0, 22)
part@3  = cut(plate, bore)
";

#[test]
fn insert_mid_timeline_keeps_downstream_face_identity() {
    let v1 = build(PLATE_V1);
    let v2 = build(PLATE_INSERTED);
    assert_all_named(&v2);

    // `part@3` stays the root with its PINNED id even though two features
    // were inserted above it; plate/bore keep ids, so part's faces do.
    assert_eq!(keys(&v1), keys(&v2), "insertion renamed downstream faces");
    let m2 = parse_model(PLATE_INSERTED, "m6-gate.pcm").unwrap();
    assert_eq!(m2.graph.root(), Some(FeatureId(3)));
    assert_eq!(m2.graph.find("plate"), Some(FeatureId(1)));
    assert_eq!(m2.graph.find("bore"), Some(FeatureId(2)));
    assert_eq!(m2.graph.find("knob"), Some(FeatureId(4))); // appended ids, no shift

    // Geometry untouched -> bit-identical centroids.
    let (f1, f2) = (face_map(&v1), face_map(&v2));
    for k in f1.keys() {
        let (_, c1, _) = f1[k];
        let (_, c2, _) = f2[k];
        assert_eq!(c1.map(f64::to_bits), c2.map(f64::to_bits), "{k} moved");
    }
}

// API-level insertion: no file involved, still id-stable.
#[test]
fn timeline_api_insert_midtree_is_stable() {
    let mut m = parse_model(PLATE_V1, "m6-gate.pcm").unwrap();
    let before = keys(&build(PLATE_V1));
    let knob = m
        .graph
        .insert_at(
            2, // BETWEEN bore and part
            "knob",
            None,
            physcad::GeoNode::Box {
                w: 4.0,
                h: 4.0,
                d: 4.0,
            },
        )
        .unwrap();
    assert_eq!(knob, FeatureId(4));
    assert_eq!(m.graph.root(), Some(FeatureId(3))); // root identity stable
    let s = csg::evaluate(&m.graph).unwrap();
    assert_all_named(&s);
    assert_eq!(keys(&s), before);
}

// ---------------------------------------------------------------------------
// Gate 4: sketch-backed extrude — walls named by sketch ENTITY, surviving
// an extrude-height edit.
// ---------------------------------------------------------------------------

fn slab_src(t: f64) -> String {
    format!(
        "\
param w = 40
param d = 20
param hb = 4
sk = newsketch()
pa = point(sk, -19.7, -10.2)
pb = point(sk, 20.3, -9.9)
pc = point(sk, 19.8, 10.1)
pd = point(sk, -20.1, 9.8)
ab = line(sk, pa, pb)
bc = line(sk, pb, pc)
cd = line(sk, pc, pd)
da = line(sk, pd, pa)
fa = fix(pa, -w / 2, -d / 2)
hab = horizontal(ab)
hbc = vertical(bc)
hcd = horizontal(cd)
hda = vertical(da)
dab = distance(pa, pb, w)
dbc = distance(pb, pc, d)
oc = point(sk, 0.2, -0.3)
foc = fix(oc, 0, 0)
hole = circle(sk, oc, 3.1)
rhole = radius(hole, hb)
slab@1 = extrude(sk, {t})
"
    )
}

#[test]
fn extrude_walls_are_named_by_sketch_entities() {
    let s = build(&slab_src(8.0));
    assert_all_named(&s);
    assert_eq!(
        keys(&s),
        vec![
            "F1:cap:bottom",
            "F1:cap:top",
            "F1:wall:ab",
            "F1:wall:bc",
            "F1:wall:cd",
            "F1:wall:da",
            "F1:wall:hole",
        ],
        "extrude face table: 2 caps + 4 line walls + 1 circle wall"
    );
    // sanity: the bore wall is the cylinder, caps the annuli
    assert!(s.face("F1:wall:hole").unwrap().area_mm2 > 2.0 * std::f64::consts::PI * 3.9 * 7.0);
    let top = s.face("F1:cap:top").unwrap();
    assert!(
        (top.centroid[2] - 8.0).abs() < 1e-9,
        "top cap sits at z = t"
    );
}

#[test]
fn extrude_height_edit_rebinds_wall_and_cap_names() {
    let v1 = build(&slab_src(8.0));
    let v2 = build(&slab_src(12.0));
    assert_all_named(&v2);
    assert_eq!(keys(&v1), keys(&v2), "height edit renamed faces");
    // top cap rises with the parameter; bottom cap stays put
    assert!((v2.face("F1:cap:top").unwrap().centroid[2] - 12.0).abs() < 1e-9);
    assert!(
        v1.face("F1:cap:top").unwrap().area_mm2 < v2.face("F1:cap:top").unwrap().area_mm2 + 1.0
    );
    assert!(centroid_close(
        v1.face("F1:cap:bottom").unwrap().centroid,
        v2.face("F1:cap:bottom").unwrap().centroid,
        1e-9
    ));
    // walls stretch, keeping their entity names
    for w in [
        "F1:wall:ab",
        "F1:wall:bc",
        "F1:wall:cd",
        "F1:wall:da",
        "F1:wall:hole",
    ] {
        let a1 = v1.face(w).unwrap();
        let a2 = v2.face(w).unwrap();
        assert_eq!(a1.num_tris, a2.num_tris, "wall {w} triangulation shifted");
        assert!(a2.area_mm2 > a1.area_mm2, "wall {w} must get taller");
    }
}

// ---------------------------------------------------------------------------
// Gate 5: revolve — surface families (cylinder, plane, cone) all bind.
// ---------------------------------------------------------------------------

fn bush_src(deg: f64) -> String {
    format!(
        "\
sk = newsketch()
e1 = point(sk, 10, 0)
e2 = point(sk, 12, 0)
e3 = point(sk, 12, 4)
e4 = point(sk, 10, 4)
f1 = line(sk, e1, e2)
f2 = line(sk, e2, e3)
f3 = line(sk, e3, e4)
f4 = line(sk, e4, e1)
a1 = point(sk, 0, -1)
a2 = point(sk, 0, 5)
ax = line(sk, a1, a2)
bush@1 = revolve(sk, ax, {deg}, 48)
"
    )
}

#[test]
fn revolve_full_turn_walls_bind_by_profile_entity() {
    let s = build(&bush_src(360.0));
    assert_all_named(&s);
    assert_eq!(
        keys(&s),
        vec![
            "F1:wall:f1", // bottom edge  -> plane perpendicular to axis
            "F1:wall:f2", // outer edge   -> cylinder r=12
            "F1:wall:f3", // top edge     -> plane perpendicular to axis
            "F1:wall:f4", // inner edge   -> cylinder r=10
        ],
        "full revolve: 4 named wall faces, no caps"
    );
}

#[test]
fn partial_revolve_adds_cap_faces_and_rebinds_on_angle_edit() {
    let v1 = build(&bush_src(90.0));
    assert_all_named(&v1);
    assert_eq!(
        keys(&v1),
        vec![
            "F1:cap:end",
            "F1:cap:start",
            "F1:wall:f1",
            "F1:wall:f2",
            "F1:wall:f3",
            "F1:wall:f4",
        ],
        "quarter revolve: 4 walls + 2 caps"
    );
    let v2 = build(&bush_src(120.0));
    assert_all_named(&v2);
    assert_eq!(keys(&v1), keys(&v2), "angle edit renamed faces");
    assert_all_named(&build(&bush_src(180.0)));
}

/// NAMING.md FM-3 pinned as an executable contract: at a 180-degree
/// sweep the start and end cap planes COINCIDE. The heuristic stays
/// deterministic (record order decides) but re-groups: `cap:start`
/// covers both half-annuli as components #0/#1 and `cap:end` gets no
/// triangles. A name surviving with new component indices is the honest
/// documented behavior, not a silent wrong answer.
#[test]
fn half_turn_cap_coincidence_is_deterministic_and_visible() {
    let v180 = build(&bush_src(180.0));
    let mut k = keys(&v180);
    k.sort();
    assert_eq!(
        k,
        vec![
            "F1:cap:start#0",
            "F1:cap:start#1",
            "F1:wall:f1",
            "F1:wall:f2",
            "F1:wall:f3",
            "F1:wall:f4",
        ]
    );
    // and deterministic: same text, same grouping, twice
    assert_eq!(face_map(&v180), face_map(&build(&bush_src(180.0))));
}

#[test]
fn revolve_cone_face_binds() {
    // quadrilateral profile with one slanted edge: (10,0) (12,0) (12,4) (10,3)
    let src = "\
sk = newsketch()
e1 = point(sk, 10, 0)
e2 = point(sk, 12, 0)
e3 = point(sk, 12, 4)
e4 = point(sk, 10, 3)
f1 = line(sk, e1, e2)
f2 = line(sk, e2, e3)
f3 = line(sk, e3, e4)
f4 = line(sk, e4, e1)
a1 = point(sk, 0, -1)
a2 = point(sk, 0, 6)
ax = line(sk, a1, a2)
cone@1 = revolve(sk, ax, 360, 64)
";
    let s = build(src);
    assert_all_named(&s);
    let want = [
        "F1:wall:f1",
        "F1:wall:f2",
        "F1:wall:f3", // slanted -> CONE face
        "F1:wall:f4",
    ];
    for k in want {
        assert!(s.face(k).is_some(), "missing {k}: got {:?}", keys(&s));
    }
    assert_eq!(s.faces.len(), 4);
}

// ---------------------------------------------------------------------------
// Gate 6: determinism — identical text, identical naming, bit for bit.
// ---------------------------------------------------------------------------

#[test]
fn naming_is_deterministic_across_rebuilds() {
    let a = build(PLATE_V1);
    let b = build(PLATE_V1);
    let (fa, fb) = (face_map(&a), face_map(&b));
    assert_eq!(fa.keys().collect::<Vec<_>>(), fb.keys().collect::<Vec<_>>());
    for k in fa.keys() {
        assert_eq!(fa[k], fb[k], "non-deterministic face {k}");
    }
    let c = build(&slab_src(8.0));
    let d = build(&slab_src(8.0));
    assert_eq!(face_map(&c), face_map(&d));
}
