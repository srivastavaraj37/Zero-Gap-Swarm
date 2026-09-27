#!/usr/bin/env bash
# Records the demo MP4 (ffmpeg from imageio-ffmpeg, no system install needed).
# Usage: scripts/record_video.sh [out.mp4] [seed] [--faults]
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${1:-results/demo_zerogap.mp4}
python3 -m zg.viz --seed "${2:-1}" --save "$OUT" --every 12 --fps 20 ${3:-}
