# Build a clean PS7-integrated LTC2208 image for the 2026 G signal analyzer.
# Release inputs are an explicit allow-list.  This intentionally prevents the
# copied AD9238/ProMax/experimental sources from entering the QSPI image merely
# because they still exist elsewhere in rtl/.

set script_path [file normalize [info script]]
set script_dir [file dirname $script_path]
set root_dir [file dirname $script_dir]
set project_name "BX71_G_ANALYZER_LTC2208_PS7"
set vivado_dir [file join $root_dir "Vivado"]
set project_dir [file join $vivado_dir $project_name]
set bd_file [file join $root_dir "ps7" "sys.bd"]
set top_name "g_analyzer_zynq_top"
set top_file [file join $root_dir "rtl_ps7" "g_analyzer_zynq_top.v"]
set release_manifest [file join $script_dir \
    "ltc2208_release_inputs.txt"]

set rtl_files [list \
    [file join $root_dir "rtl" "g_analyzer_top.v"] \
    [file join $root_dir "rtl" "stm32_spi_reg_slave.v"] \
    [file join $root_dir "rtl" "ad9708_clkgen.v"] \
    [file join $root_dir "rtl" "ltc2208_clkgen.v"] \
    [file join $root_dir "rtl" "ltc2208_cic5_fir81_decimator.v"] \
    [file join $root_dir "rtl" "ltc2208_capture_core.v"] \
    $top_file]
set constraint_files [list \
    [file join $root_dir "constraints" \
        "ltc2208_u8_to_bx71_p3_spi_gpio0.xdc"]]
set ip_files [list]


# Keep the PowerShell source fingerprint and the Vivado source list identical.
# The explicit Tcl list preserves readable source categories; this comparison
# makes either side fail closed if a future edit updates only one of them.
if {![file exists $release_manifest]} {
    error "LTC2208 release manifest is missing: $release_manifest"
}
set manifest_channel [open $release_manifest r]
set manifest_text [read $manifest_channel]
close $manifest_channel
set manifest_files [list]
foreach raw_line [split $manifest_text "\n"] {
    set relative_path [string trim $raw_line]
    if {($relative_path eq "") ||
        ([string index $relative_path 0] eq "#")} {
        continue
    }
    if {[file pathtype $relative_path] eq "absolute"} {
        error "Release manifest path must be relative: $relative_path"
    }
    if {[string match -nocase "*ad9238*" $relative_path]} {
        error "AD9238 path is forbidden in the LTC2208 release: $relative_path"
    }
    lappend manifest_files [file normalize \
        [file join $root_dir $relative_path]]
}
set declared_files [concat $rtl_files $ip_files \
    [list $bd_file] $constraint_files]
set manifest_sorted [lsort -unique $manifest_files]
set declared_sorted [lsort -unique $declared_files]
if {([llength $manifest_files] != [llength $manifest_sorted]) ||
    ($manifest_sorted ne $declared_sorted)} {
    puts "LTC2208_MANIFEST_FILES=$manifest_sorted"
    puts "LTC2208_DECLARED_FILES=$declared_sorted"
    error "Release manifest and Vivado source allow-list differ"
}
if {[lsearch -exact $rtl_files $top_file] < 0} {
    error "Release top is not part of the LTC2208 RTL allow-list: $top_file"
}

foreach required_file [concat $rtl_files $ip_files [list $bd_file] $constraint_files] {
    if {![file exists $required_file]} {
        error "Required G-analyzer source is missing: $required_file"
    }
    if {[string match -nocase "*ad9238*" [file tail $required_file]]} {
        error "AD9238 source is forbidden in the LTC2208 QSPI release: $required_file"
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
puts "LTC2208_RELEASE_RTL_COUNT=[llength $rtl_files]"
puts "LTC2208_RELEASE_IP_COUNT=[llength $ip_files]"
puts "LTC2208_RELEASE_XDC_COUNT=[llength $constraint_files]"
foreach release_file [concat $rtl_files $ip_files [list $bd_file] $constraint_files] {
    puts "LTC2208_RELEASE_INPUT=$release_file"
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
if {($dsp_count < 1) || ($dsp_count > 32)} {
    error "LTC2208 time-shared FIR datapath missing or unexpectedly large: $dsp_count"
}
set output_dir [file join $root_dir "output"]
set verification_dir [file join $output_dir "g_ltc2208_qspi_verification"]
if {[file exists $verification_dir]} {
    file delete -force $verification_dir
}
file mkdir $verification_dir

# Structural release gates: persistent boot needs PS7, this image must contain
# exactly one G capture core, and the experimental AM/FM core must be absent.
set ps7_count [llength [get_cells -hier -quiet -filter {REF_NAME == PS7}]]
set capture_count [llength [get_cells -hier -quiet -filter {REF_NAME == ltc2208_capture_core}]]
set ltc_filter_count [llength [get_cells -hier -quiet \
    -filter {REF_NAME == ltc2208_cic5_fir81_decimator}]]
set experimental_count \
    [llength [get_cells -hier -quiet -filter {REF_NAME == ad9708_wave_core}]]
puts "PS7_COUNT=$ps7_count"
puts "G_CAPTURE_CORE_COUNT=$capture_count"
puts "LTC2208_FILTER_COUNT=$ltc_filter_count"
puts "EXPERIMENTAL_AD9708_WAVE_CORE_COUNT=$experimental_count"
if {$ps7_count < 1} {
    error "PS7 was optimized out; this image cannot boot persistently from QSPI"
}
if {$capture_count != 1} {
    error "Expected exactly one G capture core, found $capture_count"
}
if {$ltc_filter_count != 1} {
    error "Expected exactly one LTC2208 CIC5/FIR81 filter, found $ltc_filter_count"
}
if {$experimental_count != 0} {
    error "Experimental AD9708 AM/FM core leaked into the G release"
}
if {[llength [get_clocks -quiet ad9708_dclock]] != 1} {
    error "AD9708 forwarded-clock constraint is missing"
}
if {[llength [get_clocks -quiet ltc2208_ch1_clk]] != 1} {
    error "LTC2208 forwarded-clock constraint is missing"
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
    -from [get_ports {ltc2208_ch1_data_i[*]}] -max_paths 1]
set adc_hold_path [get_timing_paths -delay_type min \
    -from [get_ports {ltc2208_ch1_data_i[*]}] -max_paths 1]
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
set output_bit [file join $output_dir "BX71_G_ANALYZER_LTC2208_PS7.bit"]
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
