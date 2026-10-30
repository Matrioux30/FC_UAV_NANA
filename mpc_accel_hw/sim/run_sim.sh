#!/usr/bin/env bash
# Icarus Verilog simulation of mpc_accel_axi against the golden-model test vectors.
# Usage (from anywhere):  sim/run_sim.sh [q16_16|q8_24] [extra plusargs, e.g. +NVEC=50 +STRIDE=40]
set -e
HW="$(cd "$(dirname "$0")/.." && pwd)"
FMT="${1:-q16_16}"; shift || true
cd "$HW"
[ -f "mem/$FMT/tv.hex" ] || python scripts/gen_hw_files.py
mkdir -p sim/build
iverilog -g2005 -Wall -Wno-timescale -o "sim/build/tb_$FMT.vvp" \
    -I hdl -I "mem/$FMT" -DMEM_DIR="\"mem/$FMT/\"" \
    sim/tb_mpc_accel.v hdl/mpc_accel_axi.v hdl/mpc_core.v hdl/mpc_lane.v
vvp -n "sim/build/tb_$FMT.vvp" "$@"
