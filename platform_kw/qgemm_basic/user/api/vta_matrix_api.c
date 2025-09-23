#include <stdint.h>

#include "platform_info.h"

#include "ervp_printf.h"
#include "ervp_variable_allocation.h"
#include "ervp_mmio_util.h"
#include "ervp_matrix_op_sw.h"
#include "ervp_matrix_op_transform.h"
#include "ervp_special_matrix_op.h"
#include "ervp_matrix_datatype_define.h"
#include "ervp_assert.h"
#include "ervp_caching.h"

#include "ervp_tensor.h"

#include "vta_hw_spec.h"
#include "vta_inst_api.h"
#include "vta_ctrl_api.h"
#include "vta_matrix_api.h"

// Register Map
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//       *bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved

//=====================================================================================
// INFO - Blocked GEMM test: batch=16, in_channels=16, out_channels=16, uop_comp=0
// inp_size=16 wgt_size=1 out_size=16
// batch =16 [16][16] : max
static uint32_t insns[] NOTCACHED_DATA VTA_CTRL = {
    0x00000000, 0x00000000, 0x00100001, 0x00000010,  // LOAD
    0x000001a0, 0x00000000, 0x00010010, 0x00000088,  // LOAD
    0x00000090, 0x00000000, 0x00010001, 0x00000040,  // LOAD
    0x00000140, 0x00000000, 0x00010010, 0x00000088,  // LOAD
    0x0200004a, 0x00020008, 0x00000000, 0x00000000,  // GEMM
    0x00000229, 0x00000000, 0x00010010, 0x00000088,  // STORE
    0x00000013, 0x00000000, 0x00000000, 0x00000000}; // FINISH

////// micro operation  (dst_idx, src_idx, wgt_idx)
static uint32_t uops[] NOTCACHED_DATA VTA_CTRL = {
    0x00000000,
    0x00000801,
    0x00001002,
    0x00001803,
    0x00002004,
    0x00002805,
    0x00003006,
    0x00003807,
    0x00004008,
    0x00004809,
    0x0000500a,
    0x0000580b,
    0x0000600c,
    0x0000680d,
    0x0000700e,
    0x0000780f};

#define BUFFER_SIZE 16

static uint32_t left_buffer[BUFFER_SIZE * BUFFER_SIZE] NOTCACHED_DATA VTA_DATA;
static uint32_t right_buffer[BUFFER_SIZE * BUFFER_SIZE] NOTCACHED_DATA VTA_DATA;
static const uint32_t zero_buffer[BUFFER_SIZE * BUFFER_SIZE] BIG_DATA_BSS VTA_DATA;
static uint32_t output_buffer[BUFFER_SIZE * BUFFER_SIZE] NOTCACHED_DATA VTA_DATA;
static uint32_t temp_buffer[BUFFER_SIZE * BUFFER_SIZE] NOTCACHED_DATA VTA_DATA;
static uint32_t kernel_buffer[1 * BUFFER_SIZE] NOTCACHED_DATA VTA_DATA;

const unsigned int buffer_stride = sizeof(uint32_t) * BUFFER_SIZE;

static ErvpMatrixInfo *left_buffer_info = NULL;
static ErvpMatrixInfo *right_buffer_info = NULL;
static ErvpMatrixInfo *output_buffer_info = NULL;
static ErvpMatrixInfo *temp_buffer_info = NULL;
static ErvpMatrixInfo *kernel_buffer_info = NULL;

/* (batch,16) X (16,16) = (batch,16) */
static void update_vta_inst_stride(VTAGenericInsn *insn, int input_stride, int weight_stride, int bias_stride, int output_stride)
{
  VTAMemInsn *mem_insn = insn;
  VTAGemInsn *gemm_insn = insn;
  // Load bias
  mem_insn[1].x_stride = bias_stride;
  // Load weight
  mem_insn[2].x_stride = weight_stride;
  // Load input
  mem_insn[3].x_stride = input_stride;
  // Store output
  mem_insn[5].x_stride = output_stride;
}

static void update_vta_inst_batch(VTAGenericInsn *insn, int batch)
{
  VTAMemInsn *mem_insn = insn;
  VTAGemInsn *gemm_insn = insn;
  // Load uops
  mem_insn[0].x_size = batch;
  mem_insn[0].x_stride = batch;
  // Load bias
  mem_insn[1].y_size = batch;
  // Load input
  mem_insn[3].y_size = batch;
  // Perform GEMM
  gemm_insn[4].uop_end = gemm_insn[4].uop_bgn + batch;
  // Store output
  mem_insn[5].y_size = batch;
}

static void __attribute__((constructor)) construct_vta_block()
{
  left_buffer_info = matrix_generate_info(MATRIX_DATATYPE_SINT32, BUFFER_SIZE, BUFFER_SIZE, left_buffer, NULL);
  right_buffer_info = matrix_generate_info(MATRIX_DATATYPE_SINT32, BUFFER_SIZE, BUFFER_SIZE, right_buffer, NULL);
  output_buffer_info = matrix_generate_info(MATRIX_DATATYPE_SINT32, BUFFER_SIZE, BUFFER_SIZE, output_buffer, NULL);
  temp_buffer_info = matrix_generate_info(MATRIX_DATATYPE_SINT32, BUFFER_SIZE, BUFFER_SIZE, temp_buffer, NULL);
  kernel_buffer_info = matrix_generate_info(MATRIX_DATATYPE_SINT32, 1, BUFFER_SIZE, kernel_buffer, NULL);
  update_vta_inst_stride(insns, buffer_stride, buffer_stride, buffer_stride, buffer_stride);
}

void vta_matrix_setup_buffer(const vta_hwinfo_t *const hwinfo)
{
  int num_inst = sizeof(insns) / sizeof(uint32_t) / 4;
  vta_setup_register_all(hwinfo, num_inst, insns, uops, left_buffer, right_buffer, zero_buffer, output_buffer);
}

// DO NOT REMOVE EVEN IF NOT USED
static void generate_vta_inst_for_matrix_mult(const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c)
{
  assert(a->num_col == b->num_row);
  assert(a->num_row <= 16);
  assert(a->num_col <= 16);
  assert(b->num_col <= 16);

  int batch = 16;
  int in_channels = 16;
  int out_channels = 16;
  int block = 16;
  int virtual_threads = 1;
  int uop_compression = 0;

  int ins_size = batch / block * out_channels / block * (2 + in_channels / block * 3) + 2;
  VTAGenericInsn *insn_buf = getVTABlockedGEMMInsn_block16_stride(matrix_get_stride(a),
                                                                  matrix_get_stride(b),
                                                                  matrix_get_stride(c));

  int uop_size = uop_compression ? block / VTA_BATCH * virtual_threads : block / VTA_BATCH * block / VTA_BLOCK_IN * block / VTA_BLOCK_OUT * virtual_threads;
  VTAUop *uop_buf = getGEMMUops(
      block / VTA_BATCH,
      block / VTA_BLOCK_IN,
      block / VTA_BLOCK_OUT,
      uop_compression,
      virtual_threads > 1);

  printVTADebug(insn_buf, ins_size, uop_buf, uop_size);
}

static ervp_task_wait_fx_t _vta_matrix_mult_16x16le_using_buffer(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options)
{
  // printf_function();
  assert(a->addr != left_buffer);
  assert(b->addr != right_buffer);
  assert(c->addr != output_buffer);

  // matrix_mult_size_print(a,b,c);

  ervp_task_wait_fx_t task_wait_fx = NULL;

  const int batch = a->num_row;
  const int in_channels = a->num_col;
  const int out_channels = c->num_col;

  // left_buffer_info
  left_buffer_info->num_row = batch;
  if (in_channels != 16)
  {
    // matrix_zero_sw(left_buffer_info);
    task_wait_fx = mop_mapping->matrix_zero(mop_mapping, left_buffer_info);
    task_wait_finish(task_wait_fx);
  }
  task_wait_fx = mop_mapping->matrix_copy_part(mop_mapping, a, left_buffer_info, batch, in_channels, 0);

  // right_buffer_info
  if ((in_channels != 16) || (out_channels != 16))
  {
    // matrix_zero_sw(right_buffer_info);
    task_wait_fx = mop_mapping->matrix_zero(mop_mapping, right_buffer_info);
    task_wait_finish(task_wait_fx);
  }
  task_wait_fx = mop_mapping->matrix_transpose_part(mop_mapping, b, right_buffer_info, in_channels, out_channels, 0);

  // vta start
  update_vta_inst_batch(insns, batch);
  vta_ctrl_write(hwinfo, 0x0);
  task_wait_finish(task_wait_fx);
  vta_ctrl_write(hwinfo, 0x1);
  task_wait_fx = hwinfo->wait_fx;

  output_buffer_info->num_row = batch;
  output_buffer_info->num_col = out_channels;

  // result
  task_wait_finish(task_wait_fx);
  task_wait_fx = mop_mapping->matrix_perform_postprocess(mop_mapping, output_buffer_info, c, options);
  return task_wait_fx;
}

static const int PRINT_HW = 0;

ervp_task_wait_fx_t vta_matrix_mult_16x16le(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options)
{
  assert(a->num_col == b->num_row);
  assert(a->num_row == c->num_row);
  assert(b->num_col == c->num_col);
  assert(a->num_row <= 16);
  assert(a->num_col <= 16);
  assert(b->num_col <= 16);

  ervp_task_wait_fx_t task_wait_fx = NULL;

  // matrix_print(a);

  const int A_COL_MIN = 4;
  const int B_COL_MIN = 4;

  if ((a->num_col >= A_COL_MIN) && (b->num_col >= B_COL_MIN))
  {
    if (PRINT_HW)
      printf("\nvta %d %d %d", a->num_row, a->num_col, b->num_col);
    task_wait_fx = _vta_matrix_mult_16x16le_using_buffer(mop_mapping, hwinfo, a, b, c, options);
  }
  else
  {
    if (PRINT_HW)
      printf("\nsw %d %d %d", a->num_row, a->num_col, b->num_col);
    matrix_mult_sw(a, b, c, options);
  }
  return task_wait_fx;
}

static void _vta_matrix_mult_16x16_aligned(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options)
{
  printf_function();
  // matrix_mult_size_print(a, b, c);
  assert(mop_option_has_postprocess(options) == 0);

  assert((((unsigned int)(a->addr)) % 8) == 0);
  assert((matrix_get_stride(a) % 8) == 0);
  assert(a->datatype == MATRIX_DATATYPE_SINT32);
  assert(a->num_col == 16);

  assert((((unsigned int)(c->addr)) % 8) == 0);
  assert((matrix_get_stride(c) % 8) == 0);
  assert(c->datatype == MATRIX_DATATYPE_SINT32);
  assert(c->num_col == 16);

  const int ACC_OUTPUT = mop_option_is_acc(options);
  ervp_task_wait_fx_t task_wait_fx = NULL;

  const int batch = a->num_row;
  const int in_channels = a->num_col;
  const int out_channels = c->num_col;

  task_wait_fx = mop_mapping->matrix_transpose_part(mop_mapping, b, right_buffer_info, in_channels, out_channels, 0);

  // vta start
  update_vta_inst_batch(insns, batch);
  update_vta_inst_stride(insns, matrix_get_stride(a), buffer_stride, matrix_get_stride(c), matrix_get_stride(c));
  vta_ctrl_write(hwinfo, 0x0);

  task_wait_finish(task_wait_fx);
  vta_ctrl_write(hwinfo, 0x1);
  vta_ctrl_wait(hwinfo);
}

ervp_task_wait_fx_t _vta_matrix_mult_16x16le_aligned(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options)
{
  // printf_function();
  // matrix_mult_size_print(a, b, c);
  assert(mop_option_has_postprocess(options) == 0);
  assert(a->num_col == b->num_row);
  assert(a->num_row == c->num_row);
  assert(b->num_col == c->num_col);
  assert(a->num_row <= 16);
  assert(a->num_col <= 16);
  assert(b->num_col <= 16);

  // matrix_print(a);

  const int A_COL_MIN = 16;
  const int B_COL_MIN = 16;

  if ((a->num_col >= A_COL_MIN) && (b->num_col >= B_COL_MIN))
  {
    if (PRINT_HW)
      printf("\nvta %d %d %d", a->num_row, a->num_col, b->num_col);
    _vta_matrix_mult_16x16_aligned(mop_mapping, hwinfo, a, b, temp_buffer_info, 0);
    ervp_task_wait_fx_t task_wait_fx = mop_mapping->matrix_perform_postprocess(mop_mapping, temp_buffer_info, c, options);
    task_wait_finish(task_wait_fx);
  }
  else
  {
    if (PRINT_HW)
      printf("\nsw %d %d %d", a->num_row, a->num_col, b->num_col);
    matrix_mult_sw(a, b, c, options);
  }
  return NULL;
}

/*
void vta_matrix_mult_general(ervp_mop_mapping_t *mop_mapping, const vta_hwinfo_t *const hwinfo, const ErvpMatrixInfo *a, const ErvpMatrixInfo *b, ErvpMatrixInfo *c, int options)
{
  ervp_mop_option_t mop_option;
  mop_option.value = options;
  assert((mop_option.br.performs_cliping|mop_option.br.rshift)==0);

  ErvpMatrixInfo block_a = *a;
  ErvpMatrixInfo block_b = *b;
  ErvpMatrixInfo block_c = *c;

  int block_size = hwinfo->block_size;

  ervp_mop_option_t sub_mop_option = mop_option_alloc(0);
  sub_mop_option.br.acc = 1;

  matrix_zero_sw(c);

  for (int m = 0; m < a->num_row; m += block_size)
  {
    int remain_row = a->num_row - m;
    if (remain_row >= block_size)
      remain_row = block_size;
    block_a.num_row = remain_row;
    block_c.num_row = remain_row;

    for (int n = 0; n < b->num_col; n += block_size)
    {
      int remain_col = b->num_col - n;
      if (remain_col >= block_size)
        remain_col = block_size;
      block_b.num_col = remain_col;
      block_c.num_col = remain_col;

      block_c.addr = matrix_get_element_addr(c, m, n);

      for (int k = 0; k < a->num_col; k += block_size)
      {
        int remain_product = a->num_col - k;
        if (remain_product >= block_size)
          remain_product = block_size;
        block_a.num_col = remain_product;
        block_b.num_row = remain_product;

        block_a.addr = matrix_get_element_addr(a, m, k);
        block_b.addr = matrix_get_element_addr(b, k, n);

        vta_matrix_mult_16x16le(mop_mapping, hwinfo, &block_a, &block_b, &block_c, sub_mop_option.value);
      }
    }
  }
  mop_option_free(sub_mop_option);
}
*/

#if 0
void vta_matrix_conv_block_im2col(ervp_mop_mapping_t *mop_mapping, const ErvpMatrixInfo *input_info, const ErvpMatrixInfo *kernel_info, ErvpMatrixInfo *output_info, int conv_options)
{
  // matrix_conv_size_print(input_info, kernel_info, output_info);

  ervp_task_wait_fx_t task_wait_fx = NULL;

  ervp_mconv_option_t conv_option;
  conv_option.value = conv_options;
  conv_option.br.performs_cliping = 0;
  conv_option.br.rshift = 0;

  task_wait_fx = matrix_conv2mult_im2col(mop_mapping, input_info, kernel_info, temp_buffer_info, conv_option.value);

  ervp_mop_option_t mop_option;
  mop_option.value = 0;
  mop_option.br.acc = conv_option.br.acc;
  mop_option.br.performs_cliping = conv_option.br.performs_cliping;
  mop_option.br.rshift = conv_option.br.rshift;
  task_wait_finish(task_wait_fx);
  task_wait_fx = mop_mapping->matrix_copy_part(mop_mapping, temp_buffer_info, output_info, output_info->num_row, output_info->num_col, mop_option.value);

  return task_wait_fx;
}
#endif