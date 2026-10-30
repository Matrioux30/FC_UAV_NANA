# Builds the KV260 (XCK26) design: Zynq UltraScale+ PS + SmartConnect + mpc_accel_axi, through bitstream.
# Vivado 2022.1. Run from mpc_accel_hw/:
#   vivado -mode batch -source vivado/build.tcl -tclargs q16_16            (full build)
#   vivado -mode batch -source vivado/build.tcl -tclargs q16_16 bd         (project + block design only)
#   vivado -mode batch -source vivado/build.tcl -tclargs q16_16 synth      (stop after synthesis)
# Outputs in vivado/build_<fmt>/out/: mpc_accel.bit, mpc_accel.bit.bin, mpc_accel.xsa, reports.

set fmt  [expr {[llength $argv] > 0 ? [lindex $argv 0] : "q16_16"}]
set stop [expr {[llength $argv] > 1 ? [lindex $argv 1] : "bit"}]
set hw   [file normalize [file join [file dirname [info script]] ..]]
set mem  [file join $hw mem $fmt]
if {![file exists [file join $mem coef_col.mem]]} { error "mem/$fmt missing: run python scripts/gen_hw_files.py" }
if {[string first " " $hw] >= 0} {
    error "path '$hw' contains a space; Vivado IP generation and bootgen break on it. Copy mpc_accel_hw to e.g. C:/work/"
}
source [file join $mem config.tcl]

set part     xck26-sfvc784-2LV-c
set proj     mpc_kv260_$fmt
set proj_dir [file join $hw vivado build_$fmt]
set out      [file join $proj_dir out]
set jobs     [expr {[info exists ::env(NUMBER_OF_PROCESSORS)] ? $::env(NUMBER_OF_PROCESSORS) : 4}]

# ------------------------------------------------------------------ project
create_project $proj $proj_dir -part $part -force

# KV260 board files (needed for the PS preset: DDR, MIO, clocks). Installed from the Xilinx board store if missing.
if {[llength [get_board_parts -quiet *kv260_som*]] == 0} {
    puts "INFO: installing KV260 board files from the board store"
    catch { xhub::refresh_catalog [xhub::get_xstores xilinx_board_store] }
    catch { xhub::install [xhub::get_xitems *kv260_som*] }
}
set board [lindex [lsort [get_board_parts -quiet *kv260_som*]] end]
if {$board ne ""} {
    puts "INFO: board part $board"
    set_property board_part $board [current_project]
} else {
    puts "WARNING: KV260 board part not found; PS is configured without the board preset"
}

add_files -norecurse [glob [file join $hw hdl *.v]]
add_files -norecurse [file join $hw hdl mpc_ops.vh]
set_property file_type {Verilog Header} [get_files mpc_ops.vh]
set_property is_global_include true [get_files mpc_ops.vh]
set_property include_dirs [file join $hw hdl] [current_fileset]
# $readmemh files: added to the project so the RTL can refer to them by bare file name
add_files -norecurse [glob [file join $mem *.mem]]
update_compile_order -fileset sources_1

# ------------------------------------------------------------------ block design
create_bd_design mpc_bd

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e zynq_ultra_ps_e_0]
if {$board ne ""} {
    apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} $ps
}
# HPM0_FPD only (0xA000_0000), default 128-bit width (matches the Kria boot firmware), pl_clk0 = 100 MHz, PL->PS IRQ0
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0 {1} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
    CONFIG.PSU__USE__M_AXI_GP2 {0} \
    CONFIG.PSU__FPGA_PL0_ENABLE {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
    CONFIG.PSU__USE__IRQ0 {1} \
] $ps

set acc [create_bd_cell -type module -reference mpc_accel_axi mpc_accel_0]
set_property -dict [list \
    CONFIG.FRAC_MX $FRAC_MX CONFIG.FRAC_MR $FRAC_MR CONFIG.FRAC_W $FRAC_W CONFIG.FRAC_P $FRAC_P \
    CONFIG.N_ITER_DEFAULT $N_ITER_DEFAULT \
] $acc

# Auto clocking picks pl_clk0 (the only PL clock); the automation also adds proc_sys_reset
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config [list \
    Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} Master {/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD} \
    Slave {/mpc_accel_0/s_axi} ddr_seg {Auto} intc_ip {New AXI SmartConnect} master_apm {0}] \
    [get_bd_intf_pins mpc_accel_0/s_axi]
connect_bd_net [get_bd_pins mpc_accel_0/irq] [get_bd_pins zynq_ultra_ps_e_0/pl_ps_irq0]

assign_bd_address
set seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] -filter {NAME =~ *mpc_accel*}]
set_property offset 0xA0000000 $seg
set_property range 4K $seg

validate_bd_design
regenerate_bd_layout
save_bd_design
set bd_file [get_files mpc_bd.bd]
generate_target all $bd_file
set wrapper [make_wrapper -files $bd_file -top]
add_files -norecurse $wrapper
set_property top mpc_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1
file mkdir $out
write_bd_tcl -force [file join $out mpc_bd_recreate.tcl]
if {$stop eq "bd"} { puts "INFO: block design done"; return }

# ------------------------------------------------------------------ synthesis / implementation
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "synthesis failed" }
open_run synth_1
report_utilization -file [file join $out utilization_synth.rpt]
if {$stop eq "synth"} { puts "INFO: synthesis done"; return }

launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "implementation failed" }
open_run impl_1
report_utilization    -file [file join $out utilization_impl.rpt]
report_timing_summary -file [file join $out timing_summary.rpt]
report_power          -file [file join $out power.rpt]
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "INFO: WNS = $wns ns"
if {$wns < 0} { puts "WARNING: timing not met (WNS $wns ns); see timing_summary.rpt" }

set bit [lindex [glob [file join $proj_dir $proj.runs impl_1 *.bit]] 0]
file copy -force $bit [file join $out mpc_accel.bit]
write_hw_platform -fixed -include_bit -force [file join $out mpc_accel.xsa]

# .bit.bin for the Linux FPGA manager (xmutil loadapp)
set bif [file join $out mpc_accel.bif]
set fh [open $bif w]
puts $fh "all:\n{\n    [file join $out mpc_accel.bit]\n}"
close $fh
if {[catch {exec bootgen -image $bif -arch zynqmp -process_bitstream bin -w on} msg]} {
    puts "WARNING: bootgen failed ($msg). Run manually: bootgen -image mpc_accel.bif -arch zynqmp -process_bitstream bin"
}
puts "INFO: outputs in $out"
