# Checks that this Vivado install can synthesize and implement for the KV260 device (XCK26).
#   vivado -mode batch -source vivado/check_license.tcl
# Prints LICENSE CHECK: PASS / FAIL with the reason. Runs in memory, writes nothing to the project folder.

set part xck26-sfvc784-2LV-c
puts "Vivado [version -short]"

set parts [get_parts -quiet $part]
if {[llength $parts] == 0} {
    puts "LICENSE CHECK: FAIL - part $part not installed."
    puts "  Fix: run the Xilinx installer -> 'Add Design Tools or Devices' -> SoCs -> Zynq UltraScale+ MPSoC."
    return
}
puts "part found: $parts (family [get_property FAMILY $parts])"

set tmp [file join [pwd] .lic_check_[pid]]
file mkdir $tmp
set src [file join $tmp lic_check.v]
set fh [open $src w]
puts $fh {module lic_check(input clk, input [7:0] a, input [7:0] b, output reg [15:0] p);
always @(posedge clk) p <= a * b;
endmodule}
close $fh

set ok 1
if {[catch {
    read_verilog $src
    synth_design -top lic_check -part $part -mode out_of_context
    opt_design
    place_design
} msg]} {
    set ok 0
    puts "LICENSE CHECK: FAIL - $msg"
    puts "  If the message mentions a license (Common 17-345 / 17-349): open Vivado License Manager ->"
    puts "  'Obtain License' -> Vivado ML Standard (no-cost) license, or check the node-locked/floating license."
}
file delete -force $tmp
if {$ok} { puts "LICENSE CHECK: PASS - synthesis and placement for $part work on this install." }
