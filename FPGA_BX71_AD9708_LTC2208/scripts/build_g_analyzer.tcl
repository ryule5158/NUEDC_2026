# Build the timing-clean PL image used by the 2026 G signal analyzer.
set script_path [info script]
if {[file pathtype $script_path] eq "absolute"} {
    set script_dir [file dirname $script_path]
} else {
    set script_dir [file join [pwd] [file dirname $script_path]]
}
set root_dir [file dirname $script_dir]
set project_name "BX71_G_ANALYZER_LTC2208_PL"
set project_dir [file join $root_dir "Vivado" $project_name]
set source_files [list \
    [file join $root_dir "rtl" "g_analyzer_top.v"] \
    [file join $root_dir "rtl" "stm32_spi_reg_slave.v"] \
    [file join $root_dir "rtl" "ad9708_clkgen.v"] \
    [file join $root_dir "rtl" "ltc2208_clkgen.v"] \
    [file join $root_dir "rtl" "ltc2208_cic5_fir81_decimator.v"] \
    [file join $root_dir "rtl" "ltc2208_capture_core.v"]]
set constraint_file [file join $root_dir "constraints" \
    "ltc2208_u8_to_bx71_p3_spi_gpio0.xdc"]

foreach required_file [concat $source_files [list $constraint_file]] {
    if {![file exists $required_file]} {
        error "Required pure-PL source is missing: $required_file"
    }
}
set vivado_dir [file join $root_dir "Vivado"]
file mkdir $vivado_dir
if {[file exists $project_dir]} {
    if {([file dirname $project_dir] ne $vivado_dir) ||
        ([file tail $project_dir] ne $project_name)} {
        error "Refusing to clean unexpected project directory: $project_dir"
    }
    file delete -force $project_dir
}
create_project $project_name $project_dir -part xc7z020clg400-2 -force
set_property target_language Verilog [current_project]
add_files -norecurse $source_files
add_files -fileset constrs_1 -norecurse $constraint_file
set_property include_dirs [file join $root_dir "rtl"] [get_filesets sources_1]
set_property top g_analyzer_top [get_filesets sources_1]
update_compile_order -fileset sources_1

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
if {![string match "*Complete*" $synth_status]} {
    error "G analyzer synthesis failed: $synth_status"
}
launch_runs impl_1 -to_step route_design -jobs 8
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
if {![string match "*Complete*" $impl_status]} {
    error "G analyzer implementation failed: $impl_status"
}

open_run impl_1
# This target is intentionally the PL-only timing image.  Downgrade only the
# expected ZPS7-1 check after routing; the independent QSPI release build still
# requires and structurally counts a real PS7 cell before writing its bitstream.
set zps7_check [get_drc_checks -quiet ZPS7-1]
if {[llength $zps7_check] != 1} {
    error "Expected exactly one ZPS7-1 DRC check"
}
set_property SEVERITY Warning $zps7_check

set report_dir [get_property DIRECTORY [get_runs impl_1]]
if {[llength [get_clocks -quiet ad9708_dclock]] != 1} {
    error "AD9708 idle forwarded-clock constraint is missing"
}
if {[llength [get_clocks -quiet ltc2208_ch1_clk]] != 1} {
    error "LTC2208 forwarded-clock constraint is missing"
}
set dsp_count [llength [get_cells -hier -quiet -filter {REF_NAME == DSP48E1}]]
set capture_count [llength [get_cells -hier -quiet \
    -filter {REF_NAME == ltc2208_capture_core}]]
set filter_count [llength [get_cells -hier -quiet \
    -filter {REF_NAME == ltc2208_cic5_fir81_decimator}]]
set experimental_count [llength [get_cells -hier -quiet \
    -filter {REF_NAME == ad9708_wave_core}]]
puts "G_PL_DSP48_COUNT=$dsp_count"
puts "G_PL_CAPTURE_CORE_COUNT=$capture_count"
puts "G_PL_LTC2208_FILTER_COUNT=$filter_count"
puts "G_PL_EXPERIMENTAL_CORE_COUNT=$experimental_count"
if {($dsp_count < 1) || ($dsp_count > 32)} {
    error "LTC2208 time-shared FIR datapath missing or unexpectedly large"
}
if {$capture_count != 1} {
    error "Expected exactly one LTC2208 capture core"
}
if {$filter_count != 1} {
    error "Expected exactly one LTC2208 CIC5/FIR81 filter"
}
if {$experimental_count != 0} {
    error "Experimental AD9708 wave core leaked into the G image"
}

report_timing_summary -delay_type min_max -report_unconstrained \
    -file [file join $report_dir "g_analyzer_timing_final.rpt"]
report_drc -file [file join $report_dir "g_analyzer_drc_final.rpt"]
report_cdc -details -file [file join $report_dir "g_analyzer_cdc_final.rpt"]
report_utilization -hierarchical \
    -file [file join $report_dir "g_analyzer_utilization_final.rpt"]

set drc_errors [get_drc_violations -quiet -filter {SEVERITY == Error}]
set drc_error_count [llength $drc_errors]
puts "G_PL_DRC_ERROR_COUNT=$drc_error_count"
if {$drc_error_count != 0} {
    error "Pure-PL DRC contains $drc_error_count Error violation(s)"
}
set setup_path [get_timing_paths -delay_type max -max_paths 1]
set hold_path [get_timing_paths -delay_type min -max_paths 1]

set adc_setup_path [get_timing_paths -delay_type max \
    -from [get_ports {ltc2208_ch1_data_i[*]}] -max_paths 1]
set adc_hold_path [get_timing_paths -delay_type min \
    -from [get_ports {ltc2208_ch1_data_i[*]}] -max_paths 1]
if {([llength $setup_path] == 0) || ([llength $hold_path] == 0) ||

    ([llength $adc_setup_path] == 0) || ([llength $adc_hold_path] == 0)} {
    error "G analyzer timing paths are missing"
}
set setup_slack [get_property SLACK $setup_path]
set hold_slack [get_property SLACK $hold_path]

set adc_setup_slack [get_property SLACK $adc_setup_path]
set adc_hold_slack [get_property SLACK $adc_hold_path]
puts "G_FINAL_SETUP_SLACK=$setup_slack"
puts "G_FINAL_HOLD_SLACK=$hold_slack"

puts "G_ADC_SETUP_SLACK=$adc_setup_slack"
puts "G_ADC_HOLD_SLACK=$adc_hold_slack"
if {($setup_slack < 0.0) || ($hold_slack < 0.0) ||

    ($adc_setup_slack < 0.0) || ($adc_hold_slack < 0.0)} {
    error "G analyzer routed timing failed"
}

set output_dir [file join $root_dir "output"]
set verification_dir [file join $output_dir "g_ltc2208_verification"]
file mkdir $verification_dir
foreach report_name {timing drc cdc utilization} {
    file copy -force \
        [file join $report_dir "g_analyzer_${report_name}_final.rpt"] \
        [file join $verification_dir "${report_name}_final.rpt"]
}
set bit_file [file join $report_dir "g_analyzer_top.bit"]
set output_bit [file join $output_dir "BX71_G_ANALYZER_LTC2208.bit"]
write_bitstream -force $bit_file
if {![file exists $bit_file]} {
    error "G analyzer bitstream was not generated: $bit_file"
}
file copy -force $bit_file $output_bit
puts "G_OUTPUT_BIT=$output_bit"
puts "G_ANALYZER_BUILD_PASS: $impl_status"
close_project
