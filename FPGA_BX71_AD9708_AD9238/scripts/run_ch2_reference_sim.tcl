set script_path [file normalize [info script]]
set script_dir [file dirname $script_path]
set root_dir [file dirname $script_dir]
set project_dir [file join $root_dir "Vivado" "Simulation" \
    "g_ch2_reference_sim"]

file mkdir $project_dir
create_project -force g_ch2_reference_sim $project_dir -part xc7z020clg400-2
add_files -norecurse [file join $root_dir "rtl" \
    "fir307_antialias_coeffs.vh"]
add_files -norecurse [file join $root_dir "rtl" \
    "fir307_decimator_25_q8_8.v"]
add_files -fileset sim_1 -norecurse [file join $root_dir "sim" \
    "tb_fir307_decimator_25_q8_8.v"]
set_property include_dirs [file join $root_dir "rtl"] [get_filesets sources_1]
set_property top tb_fir307_decimator_25_q8_8 [get_filesets sim_1]
set_property xsim.simulate.runtime {all} [get_filesets sim_1]
update_compile_order -fileset sim_1
launch_simulation -mode behavioral
close_sim

set simulation_log [file join $project_dir \
    "g_ch2_reference_sim.sim" "sim_1" "behav" "xsim" "simulate.log"]
if {![file exists $simulation_log]} {
    close_project
    error "CH2 reference simulation log was not generated: $simulation_log"
}
set log_handle [open $simulation_log r]
set log_text [read $log_handle]
close $log_handle
if {[string first "FIR307_DECIMATOR_25_Q8_8_TEST_PASS" $log_text] < 0} {
    close_project
    error "CH2 reference simulation did not reach PASS: $simulation_log"
}
close_project
puts "G_CH2_REFERENCE_SIM_PASS"
