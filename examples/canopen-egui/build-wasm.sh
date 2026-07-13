#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

wasm-pack build --target web --release --no-default-features
gzip -9 -f -k pkg/canopen_egui_bg.wasm
gzip -9 -f -k pkg/canopen_egui.js
