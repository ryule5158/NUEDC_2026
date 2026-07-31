# Backward-compatible entry point.  The copied AD9238 implementation used to
# live here; keeping any of it would make this innocently named script build
# the wrong ADC project.  Delegate to the explicit LTC2208 PL release instead.
set script_dir [file dirname [file normalize [info script]]]
source [file join $script_dir "build_g_analyzer.tcl"]
