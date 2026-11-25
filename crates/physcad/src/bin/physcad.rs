// physcad CLI (M3): build a model file, print readout, export STL.
use std::path::PathBuf;
use std::process::ExitCode;

use physcad::csg;
use physcad::model;
use physcad::stl;

fn usage() -> ! {
    eprintln!(
        "usage: physcad build MODEL.pcm [--stl OUT.stl]\n\
         \x20 builds the scripted model (params + expressions + CSG) and\n\
         \x20 prints volume/mass/genus/triangle readout"
    );
    std::process::exit(2);
}

fn run(args: Vec<String>) -> Result<(), String> {
    let mut it = args.into_iter();
    if it.next().as_deref() != Some("build") {
        usage();
    }
    let Some(file) = it.next() else { usage() };
    let mut stl_out: Option<PathBuf> = None;
    while let Some(a) = it.next() {
        match a.as_str() {
            "--stl" => {
                let Some(p) = it.next() else {
                    return Err("--stl requires a path".into());
                };
                stl_out = Some(PathBuf::from(p));
            }
            other => return Err(format!("unknown argument {other:?}")),
        }
    }

    let src = std::fs::read_to_string(&file).map_err(|e| format!("cannot read {file}: {e}"))?;
    let m = model::parse_model(&src, &file).map_err(|e| e.to_string())?;

    println!("model:        {file}");
    println!("features:     {}", m.graph.features.len());
    for (i, feat) in m.graph.features.iter().enumerate() {
        let name = feat.name.clone();
        let id = feat.id;
        let kind = match &feat.op {
            csg::GeoNode::Box { .. } => "box",
            csg::GeoNode::Cylinder { .. } => "cylinder",
            csg::GeoNode::Sphere { .. } => "sphere",
            csg::GeoNode::Torus { .. } => "torus",
            csg::GeoNode::Extrude { sketch, .. } => {
                println!("  #{i:<2} {id} {name:<12} extrude(sketch {sketch})");
                continue;
            }
            csg::GeoNode::Revolve { sketch, axis, .. } => {
                println!("  #{i:<2} {id} {name:<12} revolve(sketch {sketch} about {axis})");
                continue;
            }
            csg::GeoNode::Move { .. } => "move",
            csg::GeoNode::Union { .. } => "union",
            csg::GeoNode::Cut { .. } => "cut",
            csg::GeoNode::Intersect { .. } => "intersect",
        };
        println!("  #{i:<2} {id} {name:<12} {kind}");
    }

    // M4: solve every sketch in the file; diagnostics, never a panic.
    if !m.sketches.is_empty() {
        println!("sketches:     {}", m.sketches.len());
    }
    for sk in &m.sketches {
        let (_solved, rep) = physcad::sketch::solver::solve(sk);
        println!(
            "  {} ({} entities, {} constraints): {}",
            sk.name,
            sk.entities.len(),
            sk.constraints.len(),
            rep
        );
        if rep.status != physcad::sketch::solver::SolveStatus::Solved {
            eprintln!(
                "warning: sketch {:?} did not converge: {}",
                sk.name, rep.message
            );
        }
        if rep.dof > 0 {
            eprintln!(
                "note: sketch {:?} under-constrained: {} dof",
                sk.name, rep.dof
            );
        }
    }

    let Some(root_id) = m.graph.root() else {
        // sketch-only file: the readout above is the whole story
        return Ok(());
    };
    println!(
        "root:         {}",
        m.graph
            .feature(root_id)
            .map(|f| f.name.clone())
            .unwrap_or_default()
    );
    let s = csg::evaluate_with(&m.graph, &m.sketches).map_err(|e| e.to_string())?;
    println!("volume_mm3:   {:.6}", s.volume_mm3);
    println!(
        "mass_g:       {:.4}   (density {:.3} g/cm^3)",
        s.mass_g(m.density_g_cm3),
        m.density_g_cm3
    );
    println!("genus:        {}", s.genus);
    println!("triangles:    {}", s.num_tri);

    // M6: named faces — identity survives rebuilds (docs/NAMING.md).
    println!("named_faces:  {}", s.faces.len());
    for f in &s.faces {
        println!(
            "  {:<28} tris={:<7} area={:>11.4} mm^2  centroid=({:>9.3}, {:>9.3}, {:>9.3})",
            f.key, f.num_tris, f.area_mm2, f.centroid[0], f.centroid[1], f.centroid[2]
        );
    }
    let unmatched = s.faces.iter().filter(|f| !f.feature.is_real()).count();
    if unmatched > 0 {
        eprintln!(
            "warning: {unmatched} unmatched triangle group(s): the naming heuristic \
             could not bind every face — see docs/NAMING.md failure modes"
        );
    }

    if let Some(path) = stl_out {
        let bytes = stl::to_stl_binary(&s.mesh);
        std::fs::write(&path, &bytes)
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        println!("stl:          {} ({} bytes)", path.display(), bytes.len());
    }
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        usage();
    }
    match run(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
