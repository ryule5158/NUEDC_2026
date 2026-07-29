# 重新运行原AD9280采集和AD9708幅度缩放回归。

set script_path [info script]
if {[file pathtype $script_path] eq "absolute"} {
    set script_dir [file dirname $script_path]
} else {
    set script_dir [file join [pwd] [file dirname $script_path]]
}
set root_dir [file dirname $script_dir]

proc run_legacy_test {root_dir project_name top_name rtl_file tb_file} {
    set project_dir [file join $root_dir "Vivado" "Simulation" $project_name]
    file mkdir $project_dir
    create_project -force $project_name $project_dir -part xc7z020clg400-2
    if {$top_name eq "tb_ad9280_capture_core"} {
        add_files -norecurse [file join $root_dir "rtl" \
            "cic3_decimator_16.v"]
    }
    add_files -norecurse [file join $root_dir "rtl" $rtl_file]
    add_files -fileset sim_1 -norecurse [file join $root_dir "sim" $tb_file]
    set_property top $top_name [get_filesets sim_1]
    set_property xsim.simulate.runtime {all} [get_filesets sim_1]
    update_compile_order -fileset sim_1
    launch_simulation -mode behavioral
    close_sim
    close_project
}

run_legacy_test $root_dir legacy_adc_sim tb_ad9280_capture_core \
    ad9280_capture_core.v tb_ad9280_capture_core.v
run_legacy_test $root_dir legacy_dac_sim tb_ad9708_level_scaling \
    ad9708_wave_core.v tb_ad9708_level_scaling.v
