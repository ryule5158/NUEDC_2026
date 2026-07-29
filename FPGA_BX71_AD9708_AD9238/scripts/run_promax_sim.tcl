# 运行AD9238到ProMax实时数据面的自检查仿真。

set script_path [info script]
if {[file pathtype $script_path] eq "absolute"} {
    set script_dir [file dirname $script_path]
} else {
    set script_dir [file join [pwd] [file dirname $script_path]]
}
set root_dir [file dirname $script_dir]
set project_dir [file join $root_dir "Vivado" "Simulation" "promax_sim"]
file mkdir $project_dir
set testbench [file join $root_dir "sim" "tb_ad9238_promax_core.sv"]
set rtl_files [concat \
    [list [file join $root_dir "rtl" "ad9238_promax_core.sv"]] \
    [glob [file join $root_dir "rtl" "promax" "*.sv"]]]

create_project -force promax_sim $project_dir -part xc7z020clg400-2
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
set_property XPM_LIBRARIES {XPM_FIFO} [current_project]
add_files -norecurse $rtl_files
add_files -fileset sim_1 -norecurse $testbench
set_property file_type SystemVerilog [get_files [concat $rtl_files [list $testbench]]]
set_property top tb_ad9238_promax_core [get_filesets sim_1]
set_property xsim.simulate.runtime {all} [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
launch_simulation -mode behavioral
close_sim

# Vivado 2018.3在testbench调用$fatal后仍可能让批处理进程返回0。
# 因此必须在日志中看到测试末尾的唯一PASS标记，不能只相信进程退出码。
set simulation_log [file join $project_dir \
    "promax_sim.sim" "sim_1" "behav" "xsim" "simulate.log"]
if {![file exists $simulation_log]} {
    close_project
    error "ProMax simulation log was not generated: $simulation_log"
}
set log_handle [open $simulation_log r]
set log_text [read $log_handle]
close $log_handle
if {[string first "PROMAX_INTEGRATION_TEST_PASS" $log_text] < 0} {
    close_project
    error "ProMax simulation did not reach PASS; inspect: $simulation_log"
}
close_project
puts "PROMAX_SIM_PASS"
