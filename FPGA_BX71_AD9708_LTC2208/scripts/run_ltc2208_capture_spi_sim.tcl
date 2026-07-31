set script_path [file normalize [info script]]
set script_dir [file dirname $script_path]
set root_dir [file dirname $script_dir]
set project_name "g_ltc2208_capture_spi_sim"
set project_dir [file join $root_dir "Vivado" "Simulation" $project_name]

file delete -force $project_dir
create_project -force $project_name $project_dir -part xc7z020clg400-2
add_files -norecurse [file join $root_dir "rtl" \
    "ltc2208_cic5_fir81_decimator.v"]
add_files -norecurse [file join $root_dir "rtl" \
    "ltc2208_capture_core.v"]
add_files -norecurse [file join $root_dir "rtl" \
    "stm32_spi_reg_slave.v"]
add_files -fileset sim_1 -norecurse [file join $root_dir "sim" \
    "tb_ltc2208_capture_spi_core.v"]
set_property top tb_ltc2208_capture_spi_core [get_filesets sim_1]
set_property xsim.simulate.runtime {all} [get_filesets sim_1]
update_compile_order -fileset sim_1
launch_simulation -mode behavioral
close_sim

set simulation_log [file join $project_dir \
    "${project_name}.sim" "sim_1" "behav" "xsim" "simulate.log"]
if {![file exists $simulation_log]} {
    close_project
    error "LTC2208 capture/SPI simulation log was not generated: $simulation_log"
}
set log_handle [open $simulation_log r]
set log_text [read $log_handle]
close $log_handle
if {[string first "LTC2208_CAPTURE_SPI_REGRESSION_PASS" $log_text] < 0} {
    close_project
    error "LTC2208 capture/SPI regression did not reach PASS: $simulation_log"
}
close_project
puts "LTC2208_CAPTURE_SPI_SIM_PASS"
