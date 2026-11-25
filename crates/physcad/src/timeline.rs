//! M6: the feature timeline — the ordered model of *what was made, in
//! what order*, with stable identity per feature.
//!
//! This is the creation-history carrier demanded by docs/SURVEY.md §2:
//! face/edge identity across rebuilds is keyed by `FeatureId`, never by
//! traversal order or by the user-visible name. Two consequences typed
//! as data:
//!
//! - **`FeatureId`** (nonzero, monotonic, never reused) survives renames
//!   and mid-timeline insertion. `GeoNode` operands reference features by
//!   `FeatureId`, so cutting `part` after inserting a feature between its
//!   operands still points at the same two solids (M3/M4/M5 positional
//!   operands were the exact renumbering bug this kills).
//! - **Ordering is still data**: `features` is the dependency-ordered
//!   vector (operands must be defined *before* the insertion point —
//!   checked in `push`/`insert_at`), and `root` is the last feature.
//!
//! `.pcm` files may pin ids explicitly (`name@7 = ...`); an unpinned
//! feature gets `next_id` at parse time. Pinned ids are what make the
//! *file* survive an insertion mid-timeline; the editor UI must assign
//! ids at feature-creation time and never renumber (documented failure
//! mode FM-6 in docs/NAMING.md).

use std::collections::BTreeMap;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct FeatureId(pub u32);

impl FeatureId {
    /// `F0` is reserved for "unmatched face" bookkeeping in naming; real
    /// ids start at 1, so every real id sorts after it.
    pub const UNKNOWN: FeatureId = FeatureId(0);

    pub fn is_real(self) -> bool {
        self.0 >= 1
    }
}

impl std::fmt::Display for FeatureId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "F{}", self.0)
    }
}

/// One node of the geometry graph: what kind of feature and which earlier
/// features (by stable id) it consumes. Operands are always features that
/// were defined earlier in the owning `Timeline` (dependency order).
#[derive(Clone, Debug, PartialEq)]
pub enum GeoNode {
    Box {
        w: f64,
        h: f64,
        d: f64,
    },
    Cylinder {
        r: f64,
        h: f64,
        segs: usize,
    },
    Sphere {
        r: f64,
        rings: usize,
        segs: usize,
    },
    /// Torus around +Y: tube of radius `tube` swept along a circle of
    /// radius `r` in the XZ plane (matches `csg`'s cylinder axis).
    Torus {
        r: f64,
        tube: f64,
        major_segs: usize,
        minor_segs: usize,
    },
    /// M5: extrude every closed region of a (solved) sketch along +z
    /// from `offset` to `offset + height`. `profile_segs` = 0 selects
    /// chord-tolerance sampling of arcs; N > 0 forces N segments per
    /// full turn (exact-polyhedral gates). Regions union into one node.
    Extrude {
        sketch: String,
        height: f64,
        offset: f64,
        profile_segs: usize,
    },
    /// M5: revolve every closed region of a (solved) sketch about an
    /// in-plane line ENTITY (`axis`), `angle` radians in (0, 2pi].
    Revolve {
        sketch: String,
        axis: String,
        angle: f64,
        sweep_segs: usize,
        profile_segs: usize,
    },
    Move {
        node: FeatureId,
        x: f64,
        y: f64,
        z: f64,
    },
    Union {
        a: FeatureId,
        b: FeatureId,
    },
    /// `a` minus `b`.
    Cut {
        a: FeatureId,
        b: FeatureId,
    },
    Intersect {
        a: FeatureId,
        b: FeatureId,
    },
}

/// A feature: identity, user-visible name, and the typed operation.
#[derive(Clone, Debug, PartialEq)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    pub op: GeoNode,
}

/// Ordered feature timeline with stable ids. The `features` vector order
/// *is* the evaluation order; ids are the identity used by face naming
/// and by cross-feature references (`GeoNode` operands).
#[derive(Clone, Debug)]
pub struct Timeline {
    pub features: Vec<Feature>,
    by_name: BTreeMap<String, usize>,
    by_id: BTreeMap<FeatureId, usize>,
    next_id: u32,
}

impl Default for Timeline {
    fn default() -> Self {
        Self {
            features: Vec::new(),
            by_name: BTreeMap::new(),
            by_id: BTreeMap::new(),
            // ids start at F1; F0 is reserved (naming "unmatched")
            next_id: 1,
        }
    }
}

impl Timeline {
    pub fn push(&mut self, name: &str, op: GeoNode) -> Result<FeatureId, String> {
        let id = FeatureId(self.next_id);
        self.insert_with_id(self.features.len(), name, id, op)?;
        Ok(id)
    }

    /// Append a feature whose id is pinned by the file (`name@K`).
    pub fn push_with_id(
        &mut self,
        name: &str,
        id: FeatureId,
        op: GeoNode,
    ) -> Result<usize, String> {
        self.insert_with_id(self.features.len(), name, id, op)
    }

    /// Insert a feature at timeline position `pos` (0..=len). Later
    /// features keep their ids and their relative order — this is the
    /// mid-timeline-edit primitive the milestone is about. Operands must
    /// already exist AND sit before `pos` (dependency order).
    pub fn insert_at(
        &mut self,
        pos: usize,
        name: &str,
        id: Option<FeatureId>,
        op: GeoNode,
    ) -> Result<FeatureId, String> {
        let id = match id {
            Some(id) => id,
            None => FeatureId(self.next_id),
        };
        self.insert_with_id(pos, name, id, op)?;
        Ok(id)
    }

    fn insert_with_id(
        &mut self,
        pos: usize,
        name: &str,
        id: FeatureId,
        op: GeoNode,
    ) -> Result<usize, String> {
        if !id.is_real() {
            return Err(format!(
                "feature id {id} is invalid (ids start at F1; F0 is reserved)"
            ));
        }
        if pos > self.features.len() {
            return Err(format!("insert position {pos} out of range"));
        }
        if self.by_name.contains_key(name) {
            return Err(format!("node {name:?} defined twice"));
        }
        if self.by_id.contains_key(&id) {
            return Err(format!(
                "feature id {id} already used by node {:?}",
                self.features[self.by_id[&id]].name
            ));
        }
        for op_id in Self::operands(&op) {
            match self.by_id.get(&op_id) {
                Some(&p) if p < pos => {}
                Some(_) => {
                    return Err(format!(
                        "node {name} references operand {op_id} at or after its own position"
                    ));
                }
                None => {
                    return Err(format!("node {name} references undefined operand {op_id}"));
                }
            }
        }
        self.features.insert(
            pos,
            Feature {
                id,
                name: name.into(),
                op,
            },
        );
        self.next_id = self.next_id.max(id.0 + 1);
        self.reindex();
        Ok(pos)
    }

    /// Rename a feature *without* touching its id — references and face
    /// keys (which embed ids, not names) are unaffected by construction.
    pub fn rename(&mut self, id: FeatureId, new_name: &str) -> Result<(), String> {
        if self.by_name.contains_key(new_name) {
            return Err(format!("name {new_name:?} already used"));
        }
        let pos = self
            .by_id
            .get(&id)
            .copied()
            .ok_or_else(|| format!("no feature with id {id}"))?;
        self.features[pos].name = new_name.to_string();
        self.reindex();
        Ok(())
    }

    pub fn find(&self, name: &str) -> Option<FeatureId> {
        self.by_name.get(name).map(|&p| self.features[p].id)
    }

    /// Timeline position of a feature id.
    pub fn pos_of(&self, id: FeatureId) -> Option<usize> {
        self.by_id.get(&id).copied()
    }

    pub fn id_at(&self, pos: usize) -> Option<FeatureId> {
        self.features.get(pos).map(|f| f.id)
    }

    pub fn feature(&self, id: FeatureId) -> Option<&Feature> {
        self.pos_of(id).map(|p| &self.features[p])
    }

    /// The root solid is the LAST feature (unchanged M3 convention).
    pub fn root(&self) -> Option<FeatureId> {
        self.features.last().map(|f| f.id)
    }

    pub fn root_pos(&self) -> Option<usize> {
        if self.features.is_empty() {
            None
        } else {
            Some(self.features.len() - 1)
        }
    }

    fn operands(op: &GeoNode) -> Vec<FeatureId> {
        match op {
            GeoNode::Move { node, .. } => vec![*node],
            GeoNode::Union { a, b } | GeoNode::Cut { a, b } | GeoNode::Intersect { a, b } => {
                vec![*a, *b]
            }
            _ => vec![],
        }
    }

    fn reindex(&mut self) {
        self.by_name.clear();
        self.by_id.clear();
        for (pos, f) in self.features.iter().enumerate() {
            self.by_name.insert(f.name.clone(), pos);
            self.by_id.insert(f.id, pos);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const B: GeoNode = GeoNode::Box {
        w: 1.0,
        h: 1.0,
        d: 1.0,
    };

    fn timeline() -> Timeline {
        let mut t = Timeline::default();
        t.push("plate", B).unwrap();
        t.push(
            "bore",
            GeoNode::Cylinder {
                r: 1.0,
                h: 2.0,
                segs: 8,
            },
        )
        .unwrap();
        let p = t.find("plate").unwrap();
        let b = t.find("bore").unwrap();
        t.push("part", GeoNode::Cut { a: p, b }).unwrap();
        t
    }

    #[test]
    fn ids_are_sequential_and_reserved_zero_is_rejected() {
        let t = timeline();
        assert_eq!(t.find("plate"), Some(FeatureId(1)));
        assert_eq!(t.find("bore"), Some(FeatureId(2)));
        assert_eq!(t.find("part"), Some(FeatureId(3)));
        let mut u = Timeline::default();
        assert!(u.push_with_id("x", FeatureId(0), B).is_err());
    }

    #[test]
    fn insert_mid_timeline_keeps_later_ids_and_references() {
        let mut t = timeline();
        // Insert a new feature BETWEEN plate/bore and part.
        let id = t
            .insert_at(
                2,
                "notch",
                None,
                GeoNode::Box {
                    w: 2.0,
                    h: 2.0,
                    d: 2.0,
                },
            )
            .unwrap();
        assert_eq!(id, FeatureId(4)); // fresh id, NOT a shift
        assert_eq!(t.find("plate"), Some(FeatureId(1)));
        assert_eq!(t.find("bore"), Some(FeatureId(2)));
        assert_eq!(t.find("part"), Some(FeatureId(3)));
        assert_eq!(t.root(), Some(FeatureId(3))); // root identity stable
        // part's operands still resolve, and they precede it in order.
        let pos = t.pos_of(FeatureId(3)).unwrap();
        if let GeoNode::Cut { a, b } = &t.features[pos].op {
            assert_eq!((*a, *b), (FeatureId(1), FeatureId(2)));
            assert!(t.pos_of(*a).unwrap() < pos && t.pos_of(*b).unwrap() < pos);
        } else {
            panic!("part must stay a cut");
        }
    }

    #[test]
    fn insert_rejects_forward_and_self_operands() {
        let mut t = timeline();
        let p = t.find("plate").unwrap();
        // part (id 3) is at position 2; inserting at 2 a feature that
        // uses it would evaluate an operand before it exists.
        assert!(
            t.insert_at(
                2,
                "bad",
                None,
                GeoNode::Union {
                    a: p,
                    b: FeatureId(3)
                }
            )
            .is_err()
        );
    }

    #[test]
    fn rename_preserves_identity_and_blocks_collisions() {
        let mut t = timeline();
        t.rename(FeatureId(2), "hole").unwrap();
        assert_eq!(t.find("hole"), Some(FeatureId(2)));
        assert_eq!(t.find("bore"), None);
        assert!(t.rename(FeatureId(2), "plate").is_err());
        assert!(t.rename(FeatureId(99), "x").is_err());
    }

    #[test]
    fn pinned_ids_survive_file_order_gaps() {
        let mut t = Timeline::default();
        t.push_with_id(
            "part",
            FeatureId(7),
            GeoNode::Cut {
                a: FeatureId(1),
                b: FeatureId(2),
            },
        )
        .unwrap_err(); // operands not defined yet
        t.push_with_id("plate", FeatureId(1), B).unwrap();
        t.push_with_id("bore", FeatureId(2), B).unwrap();
        t.push_with_id(
            "part",
            FeatureId(7),
            GeoNode::Cut {
                a: FeatureId(1),
                b: FeatureId(2),
            },
        )
        .unwrap();
        assert_eq!(t.root(), Some(FeatureId(7)));
        assert!(t.push("late", B).is_ok_and(|id| id == FeatureId(8)));
        assert!(t.push_with_id("dup", FeatureId(1), B).is_err());
    }
}
