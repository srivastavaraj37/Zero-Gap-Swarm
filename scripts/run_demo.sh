#!/usr/bin/env bash
# Live animation of one mission. Usage: scripts/run_demo.sh [seed] [zerogap|baseline] [--faults]
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m zg.viz --seed "${1:-1}" --mode "${2:-zerogap}" ${3:-}
