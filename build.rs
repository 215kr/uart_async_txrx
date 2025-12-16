use std::env;
use std::fs;
use std::path::Path;

fn main() {
    // Determine queue capacity from Cargo features only.
    // Cargo exposes enabled features via env vars named `CARGO_FEATURE_<UPPERNAME>`.
    // Default to 256 if no feature is selected.
    let mut cap: usize = 256;

    if env::var("CARGO_FEATURE_QUEUE_32").is_ok() {
        cap = 32;
    } else if env::var("CARGO_FEATURE_QUEUE_64").is_ok() {
        cap = 64;
    } else if env::var("CARGO_FEATURE_QUEUE_128").is_ok() {
        cap = 128;
    } else if env::var("CARGO_FEATURE_QUEUE_256").is_ok() {
        cap = 256;
    } else if env::var("CARGO_FEATURE_QUEUE_512").is_ok() {
        cap = 512;
    }

    let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set");
    let dest_path = Path::new(&out_dir).join("queue_cap.rs");
    let contents = format!("pub const QUEUE_CAP: usize = {};\n", cap);
    fs::write(&dest_path, contents).expect("Unable to write queue_cap.rs");
}
