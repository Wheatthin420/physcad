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
    let s = csg::evaluate(&m.graph).map_err(|e| e.to_string())?;

    println!("model:        {file}");
    println!("nodes:        {}", m.graph.nodes.len());
    for (i, (name, node)) in m.graph.nodes.iter().enumerate() {
        let kind = match node {
            csg::GeoNode::Box { .. } => "box",
            csg::GeoNode::Cylinder { .. } => "cylinder",
            csg::GeoNode::Sphere { .. } => "sphere",
            csg::GeoNode::Move { .. } => "move",
            csg::GeoNode::Union { .. } => "union",
            csg::GeoNode::Cut { .. } => "cut",
            csg::GeoNode::Intersect { .. } => "intersect",
        };
        println!("  #{i:<2} {name:<12} {kind}");
    }
    println!("root:         {}", m.graph.nodes[m.graph.root().unwrap()].0);
    println!("volume_mm3:   {:.6}", s.volume_mm3);
    println!(
        "mass_g:       {:.4}   (density {:.3} g/cm^3)",
        s.mass_g(m.density_g_cm3),
        m.density_g_cm3
    );
    println!("genus:        {}", s.genus);
    println!("triangles:    {}", s.num_tri);

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
