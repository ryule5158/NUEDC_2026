# Build a clean PS7-integrated image for the 2026 G signal analyzer.
# The release top stays fixed, while RTL/XDC/IP inputs are discovered on every run.

set script_path [file normalize [info script]]
set script_dir [file dirname $script_path]
set root_dir [file dirname $script_dir]
set project_name "BX71_G_ANALYZER_AD9238_PS7"
set vivado_dir [file join $root_dir "Vivado"]
set project_dir [file join $vivado_dir $project_name]
set bd_file [file join $root_dir "ps7" "sys.bd"]
set top_name "g_analyzer_zynq_top"
set top_file [file join $root_dir "rtl_ps7" "g_analyzer_zynq_top.v"]

proc collect_design_files {directory patterns} {
    if {![file isdirectory $directory]} {
        return [list]
    }
    set result [list]
    foreach pattern $patterns {
        foreach candidate [glob -nocomplain -types f -directory $directory $pattern] {
            lappend result [file normalize $candidate]
        }
    }
    foreach child [glob -nocomplain -types d -directory $directory *] {
        set result [concat $result [collect_design_files $child $patterns]]
    }
    return [lsort -unique $result]
}

set rtl_patterns [list *.v *.sv *.vhd *.vhdl *.vh *.svh]
set rtl_files [list]
foreach source_dir [list \
        [file join $root_dir "rtl"] \
        [file join $root_dir "rtl_ps7"]] {
    set rtl_files [concat $rtl_files \
        [collect_design_files $source_dir $rtl_patterns]]
}
set rtl_files [lsort -unique $rtl_files]
set constraint_files [collect_design_files \
    [file join $root_dir "constraints"] [list *.xdc]]
set ip_files [collect_design_files \
    [file join $root_dir "ip"] [list *.xci *.xcix *.dcp *.edf *.edif]]

if {[llength $rtl_files] == 0} {
    error "No RTL sources were discovered under rtl/ or rtl_ps7/"
}
if {[lsearch -exact $rtl_files [file normalize $top_file]] < 0} {
    error "Release top is not part of the discovered RTL set: $top_file"
}
if {[llength $constraint_files] == 0} {
    error "No XDC constraints were discovered under constraints/"
}

foreach required_file [concat $rtl_files $ip_files [list $bd_file] $constraint_files] {
    if {![file exists $required_file]} {
        error "Required G-analyzer source is missing: $required_file"
    }
}

# A new project is created every run. This prevents stale run databases or
# accidentally retained sources from being packaged as the latest design.
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
set_property XPM_LIBRARIES {XPM_FIFO} [current_project]
set_property include_dirs [file join $root_dir "rtl"] [get_filesets sources_1]

add_files -norecurse $rtl_files
if {[llength $ip_files] > 0} {
    add_files -norecurse $ip_files
}
add_files -norecurse $bd_file
add_files -fileset constrs_1 -norecurse $constraint_files
set_property top $top_name [get_filesets sources_1]
puts "DISCOVERED_RTL_COUNT=[llength $rtl_files]"
puts "DISCOVERED_IP_COUNT=[llength $ip_files]"
puts "DISCOVERED_XDC_COUNT=[llength $constraint_files]"
foreach discovered_file [concat $rtl_files $ip_files [list $bd_file] $constraint_files] {
    puts "DISCOVERED_INPUT=$discovered_file"
}
catch { reset_target all [get_files $bd_file] }
generate_target all [get_files $bd_file]
update_compile_order -fileset sources_1

# Build the PS7 output product first to avoid Vivado 2018.3 directory races.
set_property synth_checkpoint_mode Hierarchical [get_files $bd_file]
set ip_run [get_runs -quiet sys_processing_system7_0_0_synth_1]
if {[llength $ip_run] == 0} {
    # A freshly created project does not expose the BD OOC run until it is
    # created explicitly.  Launching synth_1 first can leave the top run at
    # "Scripts Generated" while the PS7 child is still running.
    create_ip_run [get_files $bd_file]
    set ip_run [get_runs -quiet sys_processing_system7_0_0_synth_1]
}
if {[llength $ip_run] != 1} {
    error "Expected one PS7 OOC synthesis run, found [llength $ip_run]"
}
launch_runs $ip_run -jobs 1
wait_on_run $ip_run
set ip_status [get_property STATUS $ip_run]
if {![string match "*Complete*" $ip_status]} {
    error "PS7 IP synthesis failed: $ip_status"
}

launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
if {![string match "*Complete*" $synth_status]} {
    error "Synthesis failed: $synth_status"
}

launch_runs impl_1 -to_step route_design -jobs 8
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
if {![string match "*Complete*" $impl_status]} {
    error "Implementation failed: $impl_status"
}

open_run impl_1
set dsp_count [llength [get_cells -hier -quiet -filter {REF_NAME == DSP48E1}]]
puts "G_QSPI_DSP48_COUNT=$dsp_count"
if {($dsp_count < 154) || ($dsp_count > 220)} {
    error "FIR307 datapath missing or device DSP capacity exceeded: $dsp_count"
}
set output_dir [file join $root_dir "output"]
set verification_dir [file join $output_dir "g_ad9238_qspi_verification"]
if {[file exists $verification_dir]} {
    file delete -force $verification_dir
}
file mkdir $verification_dir

# Structural release gates: persistent boot needs PS7, this image must contain
# exactly one G capture core, and the experimental AM/FM core must be absent.
set ps7_count [llength [get_cells -hier -quiet -filter {REF_NAME == PS7}]]
set capture_count [llength [get_cells -hier -quiet -filter {NAME =~ */adc_capture}]]
set experimental_count \
    [llength [get_cells -hier -quiet -filter {REF_NAME == ad9708_wave_core}]]
puts "PS7_COUNT=$ps7_count"
puts "G_CAPTURE_CORE_COUNT=$capture_count"
puts "EXPERIMENTAL_AD9708_WAVE_CORE_COUNT=$experimental_count"
if {$ps7_count < 1} {
    error "PS7 was optimized out; this image cannot boot persistently from QSPI"
}
if {$capture_count != 1} {
    error "Expected exactly one G capture core, found $capture_count"
}
if {$experimental_count != 0} {
    error "Experimental AD9708 AM/FM core leaked into the G release"
}
if {[llength [get_clocks -quiet ad9708_dclock]] != 1} {
    error "AD9708 forwarded-clock constraint is missing"
}
if {[llength [get_clocks -quiet ad9238_ch1_clk]] != 1} {
    error "AD9238 forwarded-clock constraint is missing"
}

report_timing_summary -delay_type min_max -report_unconstrained \
    -file [file join $verification_dir "timing_final.rpt"]
report_drc -file [file join $verification_dir "drc_final.rpt"]
report_cdc -details -file [file join $verification_dir "cdc_final.rpt"]
report_utilization -hierarchical \
    -file [file join $verification_dir "utilization_final.rpt"]

set setup_path [get_timing_paths -delay_type max -max_paths 1]
set hold_path [get_timing_paths -delay_type min -max_paths 1]
set adc_setup_path [get_timing_paths -delay_type max \
    -from [get_ports {ad9238_ch1_data_i[*] ad9238_ch1_otr_i}] -max_paths 1]
set adc_hold_path [get_timing_paths -delay_type min \
    -from [get_ports {ad9238_ch1_data_i[*] ad9238_ch1_otr_i}] -max_paths 1]
foreach {path_name path_value} [list \
    GLOBAL_SETUP $setup_path \
    GLOBAL_HOLD $hold_path \
    ADC_SETUP $adc_setup_path \
    ADC_HOLD $adc_hold_path] {
    if {[llength $path_value] == 0} {
        error "$path_name timing path is missing"
    }
}

set setup_slack [get_property SLACK $setup_path]
set hold_slack [get_property SLACK $hold_path]
set adc_setup_slack [get_property SLACK $adc_setup_path]
set adc_hold_slack [get_property SLACK $adc_hold_path]
puts "FINAL_SETUP_SLACK=$setup_slack"
puts "FINAL_HOLD_SLACK=$hold_slack"
puts "DAC_DATA_MODE=CONSTANT_MIDSCALE_NO_DATA_TIMING_PATH"
puts "ADC_SETUP_SLACK=$adc_setup_slack"
puts "ADC_HOLD_SLACK=$adc_hold_slack"
if {($setup_slack < 0.0) || ($hold_slack < 0.0) ||
    ($adc_setup_slack < 0.0) || ($adc_hold_slack < 0.0)} {
    error "Routed timing failed"
}

set drc_errors [get_drc_violations -quiet -filter {SEVERITY == Error}]
set drc_error_count [llength $drc_errors]
puts "DRC_ERROR_COUNT=$drc_error_count"
if {$drc_error_count != 0} {
    error "DRC contains $drc_error_count Error violation(s)"
}

# The old deliverable is never copied. A new bitstream is written only after
# all release gates above have passed.
set output_bit [file join $output_dir "BX71_G_ANALYZER_AD9238_PS7.bit"]
file mkdir $output_dir
if {[file exists $output_bit]} {
    file delete -force $output_bit
}
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
write_bitstream -force $output_bit
if {![file exists $output_bit]} {
    error "Bitstream was not generated: $output_bit"
}

puts "OUTPUT_BIT=$output_bit"
puts "BUILD_PASS=$impl_status"
close_project
