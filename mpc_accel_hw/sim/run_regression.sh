#!/usr/bin/env bash
# Full 4000-vector regression for both formats, split into parallel chunks.
# Usage: sim/run_regression.sh [chunks per format, default 4]   -> logs in sim/logs/, summary at the end
set -e
HW="$(cd "$(dirname "$0")/.." && pwd)"
CHUNKS="${1:-4}"
NVEC=4000
PER=$(( (NVEC + CHUNKS - 1) / CHUNKS ))
cd "$HW"
mkdir -p sim/build sim/logs

for FMT in q16_16 q8_24; do
    [ -f "mem/$FMT/tv.hex" ] || python scripts/gen_hw_files.py
    iverilog -g2005 -Wall -Wno-timescale -o "sim/build/tb_$FMT.vvp" \
        -I hdl -I "mem/$FMT" -DMEM_DIR="\"mem/$FMT/\"" \
        sim/tb_mpc_accel.v hdl/mpc_accel_axi.v hdl/mpc_core.v hdl/mpc_lane.v
    for ((c = 0; c < CHUNKS; c++)); do
        vvp -n "sim/build/tb_$FMT.vvp" +SKIP=$((c * PER)) +NVEC=$PER > "sim/logs/${FMT}_chunk$c.log" 2>&1 &
    done
done
wait

status=0
for FMT in q16_16 q8_24; do
    pass=0; ran=0
    for ((c = 0; c < CHUNKS; c++)); do
        line=$(grep -E "^[0-9]+/[0-9]+ vectors bit-exact" "sim/logs/${FMT}_chunk$c.log" || echo "0/0")
        p=${line%%/*}; r=${line#*/}; r=${r%% *}
        pass=$((pass + p)); ran=$((ran + r))
    done
    lat=$(grep -h "latency:" sim/logs/${FMT}_chunk*.log | head -n 1)
    echo "$FMT: $pass/$ran vectors bit-exact | $lat" | tee "sim/logs/summary_$FMT.txt"
    [ "$pass" -eq "$NVEC" ] && [ "$ran" -eq "$NVEC" ] || status=1
done
exit $status
