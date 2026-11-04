if {[catch {source camera_to_dsi_display.tcl} err]} {puts "BUILD_SCRIPT_FAILED: $err"; exit 1}
puts "BUILD_STAGE_COMPLETED"
exit
