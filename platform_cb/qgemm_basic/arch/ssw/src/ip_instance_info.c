#include "platform_info.h"
#include "ervp_malloc.h"
#include "ervp_variable_allocation.h"
#include "core_dependent.h"
#include "ip_instance_info.h"

//i_dca_matrix_qgemm_control_info
static ervp_mmiox1_hwinfo_t i_dca_matrix_qgemm_control_info_static CACHED_DATA;
const ervp_mmiox1_hwinfo_t* const i_dca_matrix_qgemm_control_info = &i_dca_matrix_qgemm_control_info_static;
static void i_dca_matrix_qgemm_control_info_wait() {
	mmiox1_inst_wait_busy(i_dca_matrix_qgemm_control_info);
}
//i_dca_matrix_qgemm_info
static dca_matrix_qgemm_hwinfo_t i_dca_matrix_qgemm_info_static CACHED_DATA;
const dca_matrix_qgemm_hwinfo_t* const i_dca_matrix_qgemm_info = &i_dca_matrix_qgemm_info_static;

static void __attribute__ ((constructor)) construct_ip_instance_info()
{
	//i_dca_matrix_qgemm_control_info
	ervp_mmiox1_hwpara_t i_dca_matrix_qgemm_control_info_para;
	i_dca_matrix_qgemm_control_info_para.bw_config = 1;
	i_dca_matrix_qgemm_control_info_para.bw_status = 32;
	i_dca_matrix_qgemm_control_info_para.bw_log = 32;
	i_dca_matrix_qgemm_control_info_para.bw_inst = 512;
	i_dca_matrix_qgemm_control_info_para.bw_input = 32;
	i_dca_matrix_qgemm_control_info_para.bw_output = 32;
	i_dca_matrix_qgemm_control_info_para.config_default_value = 0;
	i_dca_matrix_qgemm_control_info_para.log_fifo_depth = 0;
	i_dca_matrix_qgemm_control_info_para.inst_fifo_depth = 4;
	i_dca_matrix_qgemm_control_info_para.input_fifo_depth = 0;
	i_dca_matrix_qgemm_control_info_para.output_fifo_depth = 0;
	mmiox1_hwinfo_elaborate(&i_dca_matrix_qgemm_control_info_para, &i_dca_matrix_qgemm_control_info_static);
	i_dca_matrix_qgemm_control_info_static.baseaddr = I_DCA_MATRIX_QGEMM_CONTROL_BASEADDR;
	i_dca_matrix_qgemm_control_info_static.wait_fx = i_dca_matrix_qgemm_control_info_wait;
	//i_dca_matrix_qgemm_info
	dca_matrix_qgemm_hwpara_t i_dca_matrix_qgemm_info_para;
	i_dca_matrix_qgemm_info_para.bw_addr = 32;
	i_dca_matrix_qgemm_info_para.ma_bw_data = 128;
	i_dca_matrix_qgemm_info_para.mb_bw_data = 128;
	i_dca_matrix_qgemm_info_para.mc_bw_data = 128;
	i_dca_matrix_qgemm_info_para.input_matrix_size = 16;
	i_dca_matrix_qgemm_info_para.weight_matrix_size = 16;
	i_dca_matrix_qgemm_info_para.output_matrix_size = 16;
	i_dca_matrix_qgemm_info_para.tensor_para = 32;
	i_dca_matrix_qgemm_info_para.bw_config = 1;
	i_dca_matrix_qgemm_info_para.bw_status = 32;
	i_dca_matrix_qgemm_info_para.bw_log = 32;
	i_dca_matrix_qgemm_info_para.bw_inst = 512;
	i_dca_matrix_qgemm_info_para.bw_input = 32;
	i_dca_matrix_qgemm_info_para.bw_output = 32;
	i_dca_matrix_qgemm_info_para.config_default_value = 0;
	i_dca_matrix_qgemm_info_para.log_fifo_depth = 0;
	i_dca_matrix_qgemm_info_para.inst_fifo_depth = 4;
	i_dca_matrix_qgemm_info_para.input_fifo_depth = 0;
	i_dca_matrix_qgemm_info_para.output_fifo_depth = 0;
	i_dca_matrix_qgemm_info_para.size_of_memorymap = 0x8000;
	i_dca_matrix_qgemm_info_para.bw_axi_data = 128;
	i_dca_matrix_qgemm_info_para.bw_axi_tid = 4;
	dca_matrix_qgemm_hwinfo_elaborate(&i_dca_matrix_qgemm_info_para,&i_dca_matrix_qgemm_info_static);
	i_dca_matrix_qgemm_info_static.mmiox_info = i_dca_matrix_qgemm_control_info;
	//
	flush_cache();
}
