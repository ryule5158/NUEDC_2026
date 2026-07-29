# 重建BX71高速AD/DA纯PL工程，并检查布局布线后的时序。

# 解析脚本位置，得到工程、约束和输出目录。
set script_path [info script]
if {[file pathtype $script_path] eq "absolute"} {
    set script_dir [file dirname $script_path]
} else {
    set script_dir [file join [pwd] [file dirname $script_path]]
}
set root_dir [file dirname $script_dir]
set project_name "BX71_AD9708_AD9238_PL"
set project_dir [file join $root_dir "Vivado" $project_name]
set project_file [file join $project_dir "${project_name}.xpr"]
set verilog_files [glob [file join $root_dir "rtl" "*.v"]]
set systemverilog_files [concat \
    [glob -nocomplain [file join $root_dir "rtl" "*.sv"]] \
    [glob -nocomplain [file join $root_dir "rtl" "promax" "*.sv"]]]

# 首次运行创建工程；后续运行直接复用已有工程。
if {![file exists $project_file]} {
    file mkdir $project_dir
    create_project $project_name $project_dir -part xc7z020clg400-2 -force
    set_property target_language Verilog [current_project]
    add_files -norecurse $verilog_files
    add_files -norecurse $systemverilog_files
    set_property file_type SystemVerilog [get_files $systemverilog_files]
    add_files -fileset constrs_1 -norecurse \
        [file join $root_dir "constraints" "acm9238_p3_spi_gpio0.xdc"]
    set_property top ad9708_spi_top [get_filesets sources_1]
    update_compile_order -fileset sources_1
} else {
    open_project $project_file
}
set_property XPM_LIBRARIES {XPM_FIFO} [current_project]

# 复用旧工程时只补入新增RTL，避免重复添加同一个源文件。
foreach rtl_file [concat $verilog_files $systemverilog_files] {
    if {[llength [get_files -quiet $rtl_file]] == 0} {
        add_files -norecurse $rtl_file
    }
}
set_property file_type SystemVerilog [get_files $systemverilog_files]
set_property top ad9708_spi_top [get_filesets sources_1]
update_compile_order -fileset sources_1

# 重新执行综合并确认综合完成。
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
if {![string match "*Complete*" $synth_status]} {
    error "Synthesis failed: $synth_status"
}

# 执行实现和位流生成，并确认实现完成。
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
if {![string match "*Complete*" $impl_status]} {
    error "Implementation failed: $impl_status"
}

# 打开已布线设计，生成时序汇总和DRC报告。
open_run impl_1
set report_dir [get_property DIRECTORY [get_runs impl_1]]
if {[llength [get_clocks -quiet ad9708_dclock]] != 1} {
    error "AD9708 forwarded-clock constraint is missing"
}
if {[llength [get_clocks -quiet ad9238_ch1_clk]] != 1} {
    error "AD9238 forwarded-clock constraint is missing"
}
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $report_dir "ad9708_timing_final.rpt"]
report_drc -file [file join $report_dir "ad9708_drc_final.rpt"]
report_cdc -details -file [file join $report_dir "ad9708_cdc_final.rpt"]
# 单独核查ProMax的50 MHz ADC域和50 MHz配置域，避免全局异步复位掩盖真实跨域结果。
report_cdc -details -from [get_clocks ad9238_ch1_clk] -to [get_clocks clk_50m] \
    -file [file join $report_dir "ad9708_cdc_adc_to_cfg.rpt"]
report_cdc -details -from [get_clocks clk_50m] -to [get_clocks ad9238_ch1_clk] \
    -file [file join $report_dir "ad9708_cdc_cfg_to_adc.rpt"]
report_utilization -hierarchical -file [file join $report_dir "ad9708_utilization_final.rpt"]

# 163个DSP48由ProMax实时档160个和原AD9708缩放3个组成。
set dsp_count [llength [get_cells -hier -quiet -filter {REF_NAME == DSP48E1}]]
puts "FINAL_DSP48_COUNT=$dsp_count"
if {$dsp_count < 163} {
    error "ProMax datapath was optimized out: DSP48 count is $dsp_count"
}

# 提取全局、DAC输出和ADC输入的最差建立/保持路径。
set setup_path [get_timing_paths -delay_type max -max_paths 1]
set hold_path [get_timing_paths -delay_type min -max_paths 1]
set dac_setup_path [get_timing_paths -delay_type max -to [get_ports {ad9708_data_o[*]}] -max_paths 1]
set dac_hold_path [get_timing_paths -delay_type min -to [get_ports {ad9708_data_o[*]}] -max_paths 1]
set adc_setup_path [get_timing_paths -delay_type max \
    -from [get_ports {ad9238_ch1_data_i[*] ad9238_ch1_otr_i}] -max_paths 1]
set adc_hold_path [get_timing_paths -delay_type min \
    -from [get_ports {ad9238_ch1_data_i[*] ad9238_ch1_otr_i}] -max_paths 1]
if {([llength $dac_setup_path] == 0) || ([llength $dac_hold_path] == 0)} {
    error "AD9708 output timing paths are missing"
}
if {([llength $adc_setup_path] == 0) || ([llength $adc_hold_path] == 0)} {
    error "AD9238 input timing paths are missing"
}
# 读取各关键路径余量并输出到构建日志。
set setup_slack [get_property SLACK $setup_path]
set hold_slack [get_property SLACK $hold_path]
set dac_setup_slack [get_property SLACK $dac_setup_path]
set dac_hold_slack [get_property SLACK $dac_hold_path]
set adc_setup_slack [get_property SLACK $adc_setup_path]
set adc_hold_slack [get_property SLACK $adc_hold_path]
puts "FINAL_SETUP_SLACK=$setup_slack"
puts "FINAL_HOLD_SLACK=$hold_slack"
puts "DAC_SETUP_SLACK=$dac_setup_slack"
puts "DAC_HOLD_SLACK=$dac_hold_slack"
puts "ADC_SETUP_SLACK=$adc_setup_slack"
puts "ADC_HOLD_SLACK=$adc_hold_slack"

# 任一关键路径余量为负即判定构建失败。
if {($setup_slack < 0.0) || ($hold_slack < 0.0) ||
    ($dac_setup_slack < 0.0) || ($dac_hold_slack < 0.0) ||
    ($adc_setup_slack < 0.0) || ($adc_hold_slack < 0.0)} {
    error "Routed timing failed"
}

# 将Vivado生成的位流复制到统一输出目录。
set bit_file [file join $report_dir "ad9708_spi_top.bit"]
set output_dir [file join $root_dir "output"]
set output_bit [file join $output_dir "BX71_AD9708_AD9238_PL.bit"]
if {![file exists $bit_file]} {
    error "Bitstream was not generated: $bit_file"
}
file mkdir $output_dir
file copy -force $bit_file $output_bit
puts "OUTPUT_BIT=$output_bit"

puts "BUILD_PASS: $impl_status"
close_project
