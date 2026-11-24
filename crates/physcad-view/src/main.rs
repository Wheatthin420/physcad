use std::path::PathBuf;

fn usage() -> ! {
    eprintln!(
        "usage: physcad-view [--capture OUT.png [--width W --height H]]\n\
         \x20 with no args: open an interactive window (needs a display)"
    );
    std::process::exit(2);
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut capture: Option<PathBuf> = None;
    let mut width = 1280u32;
    let mut height = 720u32;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--capture" => {
                let Some(p) = args.next() else {
                    eprintln!("--capture requires a path");
                    usage();
                };
                capture = Some(PathBuf::from(p));
            }
            "--width" => {
                let Some(v) = args.next() else {
                    eprintln!("--width requires a value");
                    usage();
                };
                match v.parse() {
                    Ok(n) => width = n,
                    Err(_) => {
                        eprintln!("bad --width: {v}");
                        usage();
                    }
                }
            }
            "--height" => {
                let Some(v) = args.next() else {
                    eprintln!("--height requires a value");
                    usage();
                };
                match v.parse() {
                    Ok(n) => height = n,
                    Err(_) => {
                        eprintln!("bad --height: {v}");
                        usage();
                    }
                }
            }
            "-h" | "--help" => usage(),
            other => {
                eprintln!("unknown argument: {other}");
                usage();
            }
        }
    }

    match capture {
        Some(path) => {
            let report =
                physcad_view::renderer::capture_png(&path, width, height).unwrap_or_else(|e| {
                    eprintln!("capture failed: {e}");
                    std::process::exit(1);
                });
            println!(
                "adapter: {} ({} / {})\n{}x{} -> {}\ndistinct colors: {}",
                report.adapter_name,
                report.adapter_device_type,
                report.adapter_backend,
                report.width,
                report.height,
                report.png_path,
                report.distinct_colors,
            );
        }
        None => {
            if let Err(e) = physcad_view::app::run() {
                eprintln!("viewer exited with error: {e}");
                std::process::exit(1);
            }
        }
    }
}
