set script_path [file normalize [info script]]
set root_dir [file dirname [file dirname $script_path]]
read_verilog [file join $root_dir "rtl" "fir307_decimator_25_q8_8.v"]
set_property include_dirs [file join $root_dir "rtl"] [current_fileset]
synth_design -top fir307_decimator_25_q8_8 -part xc7z020clg400-2
report_utilization
puts "G_CH2_REFERENCE_SYNTH_PASS"
