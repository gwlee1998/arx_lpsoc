source ${RVX_HWLIB_HOME}/lib_rtl/env/add_syn_source.tcl
source ${MUNOC_HW_HOME}/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/peripheral/platform_controller/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/peripheral/core_peri_group/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/peripheral/common_peri_group/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/peripheral/external_peri_group/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/peripheral/mmiox/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/core/orca_plus/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/peripheral/timer/env/add_syn_source.tcl
source ${RVX_HWLIB_HOME}/peripheral/checker/env/add_syn_source.tcl
set DCA_SOURCE_FILE ${DCA_HW_HOME}/env/add_syn_source.tcl
if {[file exist ${DCA_SOURCE_FILE}]} {
	source ${DCA_SOURCE_FILE}
}

set verilog_module_list [concat $verilog_module_list [glob ${PLATFORM_DIR}/arch/rtl/src/*.v]]
lappend verilog_include_list ${PLATFORM_DIR}/arch/rtl/include