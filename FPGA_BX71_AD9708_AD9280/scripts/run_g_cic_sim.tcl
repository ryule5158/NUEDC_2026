set script_path [info script]
if {[file pathtype $script_path] eq "absolute"} {
    set script_dir [file dirname $script_path]
} else {
    set script_dir [file join [pwd] [file dirname $script_path]]
}
set root_dir [file dirname $script_dir]
set project_dir [file join $root_dir "Vivado" "Simulation" "g_cic_sim"]
file mkdir $project_dir

create_project -force g_cic_sim $project_dir -part xc7z020clg400-2
add_files -norecurse [file join $root_dir "rtl" "cic3_decimator_16.v"]
add_files -fileset sim_1 -norecurse [file join $root_dir "sim" "tb_cic3_decimator_16.v"]
set_property top tb_cic3_decimator_16 [get_filesets sim_1]
set_property xsim.simulate.runtime {all} [get_filesets sim_1]
update_compile_order -fileset sim_1
launch_simulation -mode behavioral
close_sim

set simulation_log [file join $project_dir \
    "g_cic_sim.sim" "sim_1" "behav" "xsim" "simulate.log"]
if {![file exists $simulation_log]} {
    close_project
    error "G CIC simulation log was not generated: $simulation_log"
}
set log_handle [open $simulation_log r]
set log_text [read $log_handle]
close $log_handle
if {[string first "G_CIC3_DECIMATOR_TEST_PASS" $log_text] < 0} {
    close_project
    error "G CIC simulation did not reach PASS; inspect: $simulation_log"
}
close_project
puts "G_CIC_SIM_PASS"
