#!/usr/bin/env bash
# Run ON the KV260 (Ubuntu 22.04) from the copied deploy/ folder, with mpc_accel.bit.bin next to this script.
# Installs the accelerator as xmutil app "mpc_accel", loads it, and builds the PS test program.
set -e
cd "$(dirname "$0")"
APP=mpc_accel
FW=/lib/firmware/xilinx/$APP

[ -f $APP.bit.bin ] || { echo "missing $APP.bit.bin (vivado/build_<fmt>/out/)"; exit 1; }
command -v dtc >/dev/null || sudo apt-get install -y device-tree-compiler
command -v g++ >/dev/null || sudo apt-get install -y build-essential

dtc -@ -O dtb -o $APP.dtbo overlay/$APP.dtsi
sudo mkdir -p $FW
sudo cp $APP.bit.bin $APP.dtbo overlay/shell.json $FW/

sudo xmutil listapps
sudo xmutil unloadapp || true
sudo xmutil loadapp $APP
sudo xmutil listapps

make -C sw
echo "done. test: sudo sw/mpc_hw_test tv.hex"
