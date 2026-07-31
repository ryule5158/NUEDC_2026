set script_path [file normalize [info script]]
set script_dir [file dirname $script_path]
set root_dir [file dirname $script_dir]
set project_dir [file join $root_dir "Vivado" "Simulation" "g_ad9238_fir_sim"]

file mkdir $project_dir
create_project -force g_ad9238_fir_sim $project_dir -part xc7z020clg400-2
add_files -norecurse [file join $root_dir "rtl" "fir307_antialias_coeffs.vh"]
add_files -norecurse [file join $root_dir "rtl" "fir307_antialias_q4.v"]
add_files -fileset sim_1 -norecurse \
    [file join $root_dir "sim" "tb_fir307_antialias_q4.v"]
set_property include_dirs [file join $root_dir "rtl"] [get_filesets sources_1]
set_property top tb_fir307_antialias_q4 [get_filesets sim_1]
set_property xsim.simulate.runtime {all} [get_filesets sim_1]
update_compile_order -fileset sim_1
launch_simulation -mode behavioral
close_sim

set simulation_log [file join $project_dir \
    "g_ad9238_fir_sim.sim" "sim_1" "behav" "xsim" "simulate.log"]
if {![file exists $simulation_log]} {
    close_project
    error "G FIR simulation log was not generated: $simulation_log"
}
set log_handle [open $simulation_log r]
set log_text [read $log_handle]
close $log_handle
if {[string first "FIR307_ANTIALIAS_Q4_TEST_PASS" $log_text] < 0} {
    close_project
    error "G FIR simulation did not reach PASS; inspect: $simulation_log"
}
close_project
puts "G_FIR_SIM_PASS"