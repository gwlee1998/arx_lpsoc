#include "platform_info.h"
#include "ervp_malloc.h"
#include "ervp_matrix_op_sw.h"
#include "ervp_printf.h"
#include "dca_matrix_info.h"

#include "dca_matrix_qgemm.h"

typedef struct {
	dca_matrix_info_t mx;
	dca_matrix_info_t mw;
	dca_matrix_info_t mo;
//   unsigned int stride_m1 : 16;
//   unsigned int pad : 16;
	unsigned int ta:1;
	unsigned int tb:1;
} dca_matrix_qgemm_inst_t;

void dca_matrix_qgemm_hwinfo_elaborate(dca_matrix_qgemm_hwpara_t* hwpara, dca_matrix_qgemm_hwinfo_t* hwinfo)
{
	hwinfo->input_matrix_size = hwpara->input_matrix_size;
	hwinfo->weight_matrix_size = hwpara->weight_matrix_size;
	hwinfo->output_matrix_size = hwpara->output_matrix_size;
}

void dca_matrix_qgemm(ervp_mop_mapping_t *mop_mapping, const dca_matrix_qgemm_hwinfo_t* const hwinfo, const ErvpMatrixInfo *mx_info, const ErvpMatrixInfo *mw_info, ErvpMatrixInfo *mo_info, int qgemm_options)
{
	dca_matrix_qgemm_inst_t inst;

//  ervp_mconv_option_t conv_option;
// 	conv_option.value = conv_options;
// 	inst.stride_m1 = conv_option.br.stride_m1;
// 	inst.pad = conv_option.br.pad;
// 	inst.pad = 0; // by hkim, V250818

	dca_generate_matrix_info(mx_info, &(inst.mx));
	dca_generate_matrix_info(mw_info, &(inst.mw));
	dca_generate_matrix_info(mo_info, &(inst.mo));
	flush_cache();
	
	mmiox1_inst_push(hwinfo->mmiox_info, &inst, 1, 0);
	printf("run @ dca_matrix_qgemm.c");
	mmiox1_inst_wait_busy(hwinfo->mmiox_info);
    // vta
    // 

}