#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    canopen_egui::run_native()
}

#[cfg(target_arch = "wasm32")]
fn main() {}
