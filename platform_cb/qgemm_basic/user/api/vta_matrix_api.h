#ifndef __VTA_MATRIX_API_H__
#define __VTA_MATRIX_API_H__

#include "ervp_matrix.h"

void vta_matrix_setup_buffer(const vta_hwinfo_t *const hwinfo);
ervp_task_wait_fx_t vta_matrix_mult_16x16le(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options);
ervp_task_wait_fx_t _vta_matrix_mult_16x16le_aligned(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options);
// void vta_matrix_conv_block_im2col(ervp_mop_mapping_t *mop_mapping, const ErvpMatrixInfo *input_info, const ErvpMatrixInfo *kernel_info, ErvpMatrixInfo *output_info, int conv_options);
// void vta_matrix_mult_general(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options);

#endif
