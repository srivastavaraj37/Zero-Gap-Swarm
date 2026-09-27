#!/usr/bin/env bash
# Full reproduction: fleet sizing, 20-seed batch, fleet sweep, demo run, plots + summary.
set -euo pipefail
cd "$(dirname "$0")/.."
W=${WORKERS:-$(( $(nproc) > 2 ? $(nproc) - 2 : 1 ))}
python3 analysis/fleet_sizing.py --write results/fleet_sizing.md
python3 analysis/run_batch.py --seeds "${SEEDS:-20}" --workers "$W"
python3 analysis/run_batch.py --sweep 24,28,32,36,38,40 --sweep-seeds 3 --workers "$W"
python3 -m zg.sim --seed 1 --out results/demo_run
python3 analysis/plot_results.py
echo "done -> results/summary.md"
