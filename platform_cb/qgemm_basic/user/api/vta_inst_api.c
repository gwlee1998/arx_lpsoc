#include "ervp_printf.h"
#include "ervp_assert.h"
#include "ervp_malloc.h"

#include "vta_ervp.h"
#include "vta_inst_api.h"

VTAGenericInsn get2DLoadStoreInsn(int opcode, int type, int sram_offset, int dram_offset,
    int y_size, int x_size, int x_stride, int y_pad, int x_pad, int pop_prev_dep, int pop_next_dep,
    int push_prev_dep, int push_next_dep) {
  // Converter
  union VTAInsn converter;
  // Memory instruction initialization
  VTAMemInsn insn = {};
  insn.opcode = opcode;
  insn.pop_prev_dep = pop_prev_dep;
  insn.pop_next_dep = pop_next_dep;
  insn.push_prev_dep = push_prev_dep;
  insn.push_next_dep = push_next_dep;
  insn.memory_type = type;
  insn.sram_base = sram_offset;
  insn.dram_base = dram_offset;
  insn.y_size = y_size;
  insn.x_size = x_size;
  insn.x_stride = x_stride;
  insn.y_pad_0 = y_pad;
  insn.y_pad_1 = y_pad;
  insn.x_pad_0 = x_pad;
  insn.x_pad_1 = x_pad;
  converter.mem = insn;
  return converter.generic;
}

VTAGenericInsn get1DLoadStoreInsn(int opcode, int type, int sram_offset, int dram_offset, int size,
    int pop_prev_dep, int pop_next_dep, int push_prev_dep, int push_next_dep) {
  // Converter
  union VTAInsn converter;
  // Memory instruction initialization
  VTAMemInsn insn = {};
  insn.opcode = opcode;
  insn.pop_prev_dep = pop_prev_dep;
  insn.pop_next_dep = pop_next_dep;
  insn.push_prev_dep = push_prev_dep;
  insn.push_next_dep = push_next_dep;
  insn.memory_type = type;
  insn.sram_base = sram_offset;
  insn.dram_base = dram_offset;
  insn.y_size = 1;
  insn.x_size = size;
  insn.x_stride = size;
  insn.y_pad_0 = 0;
  insn.y_pad_1 = 0;
  insn.x_pad_0 = 0;
  insn.x_pad_1 = 0;
  converter.mem = insn;
  return converter.generic;
}

VTAGenericInsn getGEMMInsn(int uop_offset, int batch, int in_feat, int out_feat,
    int uop_compression, int pop_prev_dep, int pop_next_dep, int push_prev_dep,
    int push_next_dep) {
  // Converter
  union VTAInsn converter;
  // GEMM instruction initialization
  VTAGemInsn insn;
  insn.opcode = VTA_OPCODE_GEMM;
  insn.pop_prev_dep = pop_prev_dep;
  insn.pop_next_dep = pop_next_dep;
  insn.push_prev_dep = push_prev_dep;
  insn.push_next_dep = push_next_dep;
  insn.reset_reg = 0;
  if (!uop_compression) {
    insn.uop_bgn = uop_offset;
    insn.uop_end = uop_offset + batch * in_feat * out_feat;
    insn.iter_out = 1;
    insn.iter_in = 1;
    insn.dst_factor_out = 0;
    insn.src_factor_out = 0;
    insn.wgt_factor_out = 0;
    insn.dst_factor_in = 0;
    insn.src_factor_in = 0;
    insn.wgt_factor_in = 0;
  } else {
    insn.uop_bgn = uop_offset;
    insn.uop_end = uop_offset + batch;
    insn.iter_out = in_feat;
    insn.iter_in = out_feat;
    insn.dst_factor_out = 0;
    insn.src_factor_out = 1;
    insn.wgt_factor_out = 1;
    insn.dst_factor_in = 1;
    insn.src_factor_in = 0;
    insn.wgt_factor_in = in_feat;
  }
  converter.gemm = insn;
  return converter.generic;
}

VTAGenericInsn getFinishInsn(int pop_prev, int pop_next) {
  // Converter
  union VTAInsn converter;
  // GEMM instruction initialization
  VTAGemInsn insn;
  insn.opcode = VTA_OPCODE_FINISH;
  insn.pop_prev_dep = pop_prev;
  insn.pop_next_dep = pop_next;
  insn.push_prev_dep = 0;
  insn.push_next_dep = 0;
  insn.reset_reg = 0;
  insn.uop_bgn = 0;
  insn.uop_end = 0;
  insn.iter_out = 0;
  insn.iter_in = 0;
  insn.dst_factor_out = 0;
  insn.src_factor_out = 0;
  insn.wgt_factor_out = 0;
  insn.dst_factor_in = 0;
  insn.src_factor_in = 0;
  insn.wgt_factor_in = 0;
  converter.gemm = insn;
  return converter.generic;
}

VTAGenericInsn getALUInsn(int opcode, int vector_size, int use_imm, int imm, int uop_compression,
    int pop_prev_dep, int pop_next_dep, int push_prev_dep, int push_next_dep) {
  // Converter
  union VTAInsn converter;
  // Memory instruction initialization
  VTAAluInsn insn = {};
  insn.opcode = VTA_OPCODE_ALU;
  insn.pop_prev_dep = pop_prev_dep;
  insn.pop_next_dep = pop_next_dep;
  insn.push_prev_dep = push_prev_dep;
  insn.push_next_dep = push_next_dep;
  insn.reset_reg = 0;
  if (!uop_compression) {
    insn.uop_bgn = 0;
    insn.uop_end = vector_size;
    insn.iter_out = 1;
    insn.iter_in = 1;
    insn.dst_factor_out = 0;
    insn.src_factor_out = 0;
    insn.dst_factor_in = 0;
    insn.src_factor_in = 0;
    insn.alu_opcode = opcode;
    insn.use_imm = use_imm;
    insn.imm = imm;
  } else {
    insn.uop_bgn = 0;
    insn.uop_end = 1;
    insn.iter_out = 1;
    insn.iter_in = vector_size;
    insn.dst_factor_out = 0;
    insn.src_factor_out = 0;
    insn.dst_factor_in = 1;
    insn.src_factor_in = 1;
    insn.alu_opcode = opcode;
    insn.use_imm = use_imm;
    insn.imm = imm;
  }
  converter.alu = insn;
  return converter.generic;
}

VTAGenericInsn *getVTAALUInsn(int opcode, int batch, int vector_size, int uop_compression)
{
  // Some assertions
  assert(batch % VTA_BATCH == 0);
  assert(vector_size % VTA_BLOCK_OUT == 0);
  //printf("=====================================================================================\n");
  //printf("INFO - ALU test of %s: batch=%d, vector_size=%d, uop_compression=%d\n",
  //  getOpcodeString(opcode, 0), batch, vector_size, uop_compression);

  // Instruction count
  int ins_size = 3 * batch / VTA_BATCH + 2;
  // Micro op count
  int uop_size = uop_compression ? 1 : vector_size / VTA_BLOCK_OUT;
  // Input/output elements in each transfer
  int tx_size = vector_size / VTA_BLOCK_OUT;
  // Number of input sets to be generated
  int input_sets = 2;
  // Make sure we don't exceed buffer bounds
  assert(uop_size <= VTA_UOP_BUFF_DEPTH);
  assert(tx_size * input_sets <= VTA_ACC_BUFF_DEPTH);

  // Initialize instructions
  VTAGenericInsn *insn_buf = (VTAGenericInsn *)calloc(ins_size, sizeof(VTAGenericInsn));
  int insn_idx = 0;
  insn_buf[insn_idx++] =
      get1DLoadStoreInsn(VTA_OPCODE_LOAD, VTA_MEM_ID_UOP, 0, 0, uop_size, 0, 0, 0, 0);
  for (int b = 0; b < batch; b += VTA_BATCH) {
    insn_buf[insn_idx++] = get2DLoadStoreInsn(
        VTA_OPCODE_LOAD,                   // opcode
        VTA_MEM_ID_ACC,                    // vector size
        0,                                 // sram offset
        b / VTA_BATCH * tx_size * input_sets,  // dram offset
        1,                                 // y size
        tx_size * input_sets,              // x size
        tx_size * input_sets,              // x stride
        0,                                 // y pad
        0,                                 // x pad
        0,                                 // pop prev dep
        b > 0,                             // pop next dep
        0,                                 // push prev dep
        0);                                // push next dep
    insn_buf[insn_idx++] = getALUInsn(
        opcode,                            // opcode
        tx_size,                           // vector size
        0,                                 // use imm
        0,                                 // imm
        uop_compression,                   // uop compression
        0,                                 // pop prev dep
        0,                                 // pop next dep
        0,                                 // push prev dep
        1);                                // push next dep
    insn_buf[insn_idx++] = get2DLoadStoreInsn(
        VTA_OPCODE_STORE,                  // opcode
        VTA_MEM_ID_OUT,                    // vector size
        0,                                 // sram offset
        b / VTA_BATCH * tx_size,           // dram offset
        1,                                 // y size
        tx_size,                           // x size
        tx_size,                           // x stride
        0,                                 // y pad
        0,                                 // x pad
        1,                                 // pop prev dep
        0,                                 // pop next dep
        1,                                 // push prev dep
        0);                                // push next dep
  }
  // Finish
  insn_buf[insn_idx++] = getFinishInsn(0, 1);

  return insn_buf;
}

VTAUop * getMapALUUops(int vector_size, int uop_compression) {
  // Derive the total uop size
  int uop_size = (uop_compression) ? 1 : vector_size;

  // Allocate buffer
  VTAUop *uop_buf = (VTAUop *)(calloc(uop_size, sizeof(VTAUop)));

  if (!uop_compression) {
    for (int i = 0; i < vector_size; i++) {
      uop_buf[i].dst_idx = i;
      uop_buf[i].src_idx = vector_size + i;
    }
  } else {
    uop_buf[0].dst_idx = 0;
    uop_buf[0].src_idx = vector_size;
  }

  return uop_buf;
}

VTAGenericInsn *getVTAGEMMInsn(int batch, int in_channels, int out_channels, int uop_compression)
{
  assert(batch % VTA_BATCH == 0);
  assert(in_channels % VTA_BLOCK_IN == 0);
  assert(out_channels % VTA_BLOCK_OUT == 0);

  // printf("=====================================================================================\n");
  // printf("INFO - Blocked GEMM test: batch=%d, in_channels=%d, out_channels=%d, uop_comp=%d\n",
  //        batch, in_channels, out_channels, uop_compression);

  int ins_size = 7;
  int uop_size = uop_compression ?
      batch / VTA_BATCH :
      batch / VTA_BATCH * in_channels / VTA_BLOCK_IN * out_channels / VTA_BLOCK_OUT;
  int inp_size = batch / VTA_BATCH * in_channels / VTA_BLOCK_IN;
  int wgt_size = in_channels / VTA_BLOCK_IN * out_channels / VTA_BLOCK_OUT;
  int out_size = batch / VTA_BATCH * out_channels / VTA_BLOCK_OUT;

  // if(wgt_size > VTA_WGT_BUFF_DEPTH)
  // {
  //   printf("uop_size %d\n", uop_size);
  //   printf("batch %d\n", batch);
  //   printf("in_channels %d\n", in_channels);
  //   printf("out_channels %d\n", out_channels);
  // printf("%d %d\n", inp_size, VTA_INP_BUFF_DEPTH);
  // printf("%d %d\n", wgt_size, VTA_WGT_BUFF_DEPTH);
  // printf("%d %d\n", out_size, VTA_ACC_BUFF_DEPTH);
  // }
  if(out_size > VTA_ACC_BUFF_DEPTH)
    printf("(%d > %d) %d x %d x %d \n", out_size, VTA_ACC_BUFF_DEPTH, batch, in_channels, out_channels);
  if (wgt_size > VTA_WGT_BUFF_DEPTH)
    printf("(%d > %d) %d x %d x %d \n", wgt_size, VTA_WGT_BUFF_DEPTH, batch, in_channels, out_channels);
  assert(uop_size <= VTA_UOP_BUFF_DEPTH);
  assert(inp_size <= VTA_INP_BUFF_DEPTH);
  assert(wgt_size <= VTA_WGT_BUFF_DEPTH);
  assert(out_size <= VTA_ACC_BUFF_DEPTH);

  // printf("%d %d\n", uop_size, VTA_UOP_BUFF_DEPTH);
  // printf("%d %d\n", inp_size, VTA_INP_BUFF_DEPTH);
  // printf("%d %d\n", wgt_size, VTA_WGT_BUFF_DEPTH);
  // printf("%d %d\n", out_size, VTA_ACC_BUFF_DEPTH);

  VTAGenericInsn *insn_buf = (VTAGenericInsn *)calloc(ins_size, sizeof(VTAGenericInsn));

  int insn_idx = 0;

  // Load uops
  insn_buf[insn_idx++] = get1DLoadStoreInsn(
      VTA_OPCODE_LOAD,
      VTA_MEM_ID_UOP,
      0,
      0,
      uop_size,
      0,
      0,
      0,
      0);
  // Load bias
  insn_buf[insn_idx++] = get1DLoadStoreInsn(
      VTA_OPCODE_LOAD,                                    // opcode
      VTA_MEM_ID_ACC,                                     // type
      0,                                                  // sram offset
      0,                                                  // dram offset
      out_size,                                           // size
      0,                                                  // pop prev dep
      0,                                                  // pop next dep
      1,                                                  // push prev dep
      0);                                                 // push next dep
  // Load weight block (pop next)
  insn_buf[insn_idx++] = get1DLoadStoreInsn(
      VTA_OPCODE_LOAD,                                    // opcode
      VTA_MEM_ID_WGT,                                     // type
      0,                                                  // sram offset
      0,                                                  // dram offset
      wgt_size,                                           // size
      0,                                                  // pop prev dep
      1,                                                  // pop next dep
      0,                                                  // push prev dep
      0);                                                 // push next dep
  // Load input block (push next)
  insn_buf[insn_idx++] = get1DLoadStoreInsn(
      VTA_OPCODE_LOAD,                                    // opcode
      VTA_MEM_ID_INP,                                     // type
      0,                                                  // sram offset
      0,                                                  // dram offset
      inp_size,                                           // size
      0,                                                  // pop prev dep
      0,                                                  // pop next dep
      0,                                                  // push prev dep
      1);                                                 // push next dep
  // Perform GEMM (pop prev, push prev if not last, push next if last)
  insn_buf[insn_idx++] = getGEMMInsn(
      0,                                                  // uop offset
      batch / VTA_BATCH,                                  // batch
      in_channels / VTA_BLOCK_IN,                         // in_channels
      out_channels / VTA_BLOCK_OUT,                       // out_channels
      uop_compression,                                    // uop_compression
      1,                                                  // pop_prev_dep
      0,                                                  // pop_next_dep
      0,                                                  // push prev dep
      1);                                                 // push_next_dep
  // Store output block (pop prev, push prev if not last)
  insn_buf[insn_idx++] = get1DLoadStoreInsn(
      VTA_OPCODE_STORE,                                   // opcode
      VTA_MEM_ID_OUT,                                     // type
      0,                                                  // sram offset
      0,                                                  // dram offset
      out_size,                                           // size
      1,                                                  // pop prev dep
      0,                                                  // pop next dep
      1,                                                  // push prev dep
      0);                                                 // push next dep
  // Finish
  insn_buf[insn_idx++] = getFinishInsn(0, 1);

  return insn_buf;
}

VTAGenericInsn *getVTABlockedGEMMInsn(int batch, int in_channels, int out_channels, int block, int uop_compression, int virtual_threads)
{
  // Some assertions
  assert(block % VTA_BLOCK_IN == 0);
  assert(block % VTA_BLOCK_OUT == 0);
  assert(block % VTA_BATCH == 0);
  assert(in_channels % block == 0);
  assert(out_channels % block == 0);
  assert(batch % block == 0);

  printf("=====================================================================================\n");
  printf("INFO - Blocked GEMM test: batch=%d, in_channels=%d, out_channels=%d, block=%d, uop_comp=%d, vt=%d\n",
         batch, in_channels, out_channels, block, uop_compression, virtual_threads);

  // Input/output channels
  int in_feat = in_channels;
  int out_feat = out_channels;
  // Derive number of elements that need to be loaded/stored
  int ins_size = batch / block * out_feat / block * (2 + in_feat / block * 3) + 2;
  int uop_size = uop_compression ?
      block / VTA_BATCH * virtual_threads :
      block / VTA_BATCH * block / VTA_BLOCK_IN * block / VTA_BLOCK_OUT * virtual_threads;
  int inp_size = batch / VTA_BATCH * in_feat / VTA_BLOCK_IN;
  int wgt_size = in_feat / VTA_BLOCK_IN * out_feat / VTA_BLOCK_OUT;
  int out_size = batch / VTA_BATCH * out_feat / VTA_BLOCK_OUT;
  // Blocked buffer sizes (in terms of elements)
  int inp_block_size = block / VTA_BATCH * block / VTA_BLOCK_IN;
  int wgt_block_size = block / VTA_BLOCK_IN * block / VTA_BLOCK_OUT;
  int out_block_size = block / VTA_BATCH * block / VTA_BLOCK_OUT;
  // Make sure we don't exceed buffer bounds
  assert(uop_size <= VTA_UOP_BUFF_DEPTH);
  assert(inp_block_size <= VTA_INP_BUFF_DEPTH);
  assert(wgt_block_size <= VTA_WGT_BUFF_DEPTH);
  assert(out_block_size <= VTA_ACC_BUFF_DEPTH);

  VTAGenericInsn *insn_buf = (VTAGenericInsn *)calloc(ins_size, sizeof(VTAGenericInsn));
  int insn_idx = 0;

  // Load uops
  insn_buf[insn_idx++] = get1DLoadStoreInsn(VTA_OPCODE_LOAD,
                                            VTA_MEM_ID_UOP,
                                            0,
                                            0,
                                            uop_size,
                                            0,
                                            0,
                                            0,
                                            0);

  // Iterate over batch blocks
  for (int i = 0; i < batch; i += block) {
    // Iterate over output channel blocks
    for (int j = 0; j < out_feat; j += block) {
      // Load bias block (pop next if not first, push prev)
      insn_buf[insn_idx++] = get2DLoadStoreInsn(
          VTA_OPCODE_LOAD,                                    // opcode
          VTA_MEM_ID_ACC,                                     // type
          0,                                                  // sram offset
          (i / VTA_BATCH * out_feat + j) / VTA_BLOCK_OUT,     // dram offset
          block / VTA_BATCH,                                  // y size
          block / VTA_BLOCK_OUT,                              // x size
          out_feat / VTA_BLOCK_OUT,                           // x stride
          0,                                                  // y pad
          0,                                                  // x pad
          0,                                                  // pop prev dep
          (i > 0 || j > 0),                                   // pop next dep
          (virtual_threads == 1),                             // push prev dep
          0);                                                 // push next dep
      // Iterate over input channel blocks
      for (int k = 0; k < in_feat; k += block * virtual_threads) {
        for (int l = 0; l < block * virtual_threads; l += block) {
          // Derive dependence flags
          int pop = (virtual_threads == 1) ?
              1 :
              (i > 0 || j > 0 || k > 0 || l > 0) && (k + l != block * virtual_threads - block);
          int push_prev = (virtual_threads == 1) ?
              ((k + l) != in_feat - block) :
              ((k + l) != in_feat - virtual_threads * block) &&
              (
                  (k + l != in_feat - block) ||
                  (j != out_feat - block) ||
                  (i != batch - block));
          int push_next = (k + l == in_feat - block);
          // Load weight block (pop next)
          insn_buf[insn_idx++] = get2DLoadStoreInsn(
              VTA_OPCODE_LOAD,                                // opcode
              VTA_MEM_ID_WGT,                                 // type
              l / VTA_BLOCK_IN * block / VTA_BLOCK_OUT,       // sram offset
              (j / VTA_BLOCK_OUT * in_feat + k + l) / VTA_BLOCK_IN,  // dram offset
              // (j * in_feat + k + l) / VTA_BLOCK_IN,  // dram offset
              block / VTA_BLOCK_OUT,                          // y size
              block / VTA_BLOCK_IN,                           // x size
              in_feat / VTA_BLOCK_IN,                         // x stride
              0,                                              // y pad
              0,                                              // x pad
              0,                                              // pop prev dep
              pop,                                            // pop next dep
              0,                                              // push prev dep
              0);                                             // push next dep
          // Load input block (push next)
          insn_buf[insn_idx++] = get2DLoadStoreInsn(
              VTA_OPCODE_LOAD,                                // opcode
              VTA_MEM_ID_INP,                                 // type
              l / VTA_BLOCK_IN * block / VTA_BATCH,           // sram offset
              (i / VTA_BATCH * in_feat + k + l) / VTA_BLOCK_IN,  // dram offset
              block / VTA_BATCH,                              // y size
              block / VTA_BLOCK_IN,                           // x size
              in_feat / VTA_BLOCK_IN,                         // x stride
              0,                                              // y pad
              0,                                              // x pad
              0,                                              // pop prev dep
              0,                                              // pop next dep
              0,                                              // push prev dep
              1);                                             // push next dep
          // Perform GEMM (pop prev, push prev if not last, push next if last)
          insn_buf[insn_idx++] = getGEMMInsn(
              l / block * uop_size / virtual_threads,         // uop offset
              block / VTA_BATCH,                              // batch
              block / VTA_BLOCK_IN,                           // in_feat
              block / VTA_BLOCK_OUT,                          // out_feat
              uop_compression,                                // uop_compression
              1,                                              // pop_prev_dep
              0,                                              // pop_next_dep
              push_prev,                                      // push prev dep
              push_next);                                     // push_next_dep
        }
      }
      // Store output block (pop prev, push prev if not last)
      insn_buf[insn_idx++] = get2DLoadStoreInsn(
          VTA_OPCODE_STORE,                                   // opcode
          VTA_MEM_ID_OUT,                                     // type
          0,                                                  // sram offset
          (i / VTA_BATCH * out_feat + j) / VTA_BLOCK_OUT,     // dram offset
          block / VTA_BATCH,                                  // y size
          block / VTA_BLOCK_OUT,                              // x size
          // out_feat / VTA_BLOCK_OUT,                           // x stride
          out_feat / VTA_BLOCK_OUT * 4,                           // x stride
          0,                                                  // y pad
          0,                                                  // x pad
          1,                                                  // pop prev dep
          0,                                                  // pop next dep
          1,                                                  // pop prev dep
          0);                                                 // push next dep
    }
  }
  // Finish
  insn_buf[insn_idx++] = getFinishInsn(0, 1);

  return insn_buf;
}

VTAGenericInsn *getVTABlockedGEMMInsn_block16_stride(int input_stride, int weight_stride, int output_stride)
{
  int batch = 16;
  int in_channels = 16;
  int out_channels = 16;
  int block = 16;
  int uop_compression = 0;
  int virtual_threads = 1;
  // Some assertions
  assert(block % VTA_BLOCK_IN == 0);
  assert(block % VTA_BLOCK_OUT == 0);
  assert(block % VTA_BATCH == 0);
  assert(in_channels % block == 0);
  assert(out_channels % block == 0);
  assert(batch % block == 0);

#if 0
  printf("=====================================================================================\n");
  printf("INFO - Blocked GEMM inst gen: batch=%d, in_channels=%d, out_channels=%d, block=%d, uop_comp=%d, vt=%d\n",
         batch, in_channels, out_channels, block, uop_compression, virtual_threads);
#endif

  // Input/output channels
  int in_feat = in_channels;
  int out_feat = out_channels;
  // Derive number of elements that need to be loaded/stored
  int ins_size = batch / block * out_feat / block * (2 + in_feat / block * 3) + 2;
  int uop_size = uop_compression ?
      block / VTA_BATCH * virtual_threads :
      block / VTA_BATCH * block / VTA_BLOCK_IN * block / VTA_BLOCK_OUT * virtual_threads;
  int inp_size = batch / VTA_BATCH * in_feat / VTA_BLOCK_IN;
  int wgt_size = in_feat / VTA_BLOCK_IN * out_feat / VTA_BLOCK_OUT;
  int out_size = batch / VTA_BATCH * out_feat / VTA_BLOCK_OUT;
  // Blocked buffer sizes (in terms of elements)
  int inp_block_size = block / VTA_BATCH * block / VTA_BLOCK_IN;
  int wgt_block_size = block / VTA_BLOCK_IN * block / VTA_BLOCK_OUT;
  int out_block_size = block / VTA_BATCH * block / VTA_BLOCK_OUT;
  // Make sure we don't exceed buffer bounds
  assert(uop_size <= VTA_UOP_BUFF_DEPTH);
  assert(inp_block_size <= VTA_INP_BUFF_DEPTH);
  assert(wgt_block_size <= VTA_WGT_BUFF_DEPTH);
  assert(out_block_size <= VTA_ACC_BUFF_DEPTH);

  VTAGenericInsn *insn_buf = (VTAGenericInsn *)calloc(ins_size, sizeof(VTAGenericInsn));
  int insn_idx = 0;

  // Load uops
  insn_buf[insn_idx++] = get1DLoadStoreInsn(VTA_OPCODE_LOAD,
                                            VTA_MEM_ID_UOP,
                                            0,
                                            0,
                                            uop_size,
                                            0,
                                            0,
                                            0,
                                            0);
  //printf("output_stride: %d\n", output_stride);
  // Iterate over batch blocks
  for (int i = 0; i < batch; i += block) {
    // Iterate over output channel blocks
    for (int j = 0; j < out_feat; j += block) {
      // Load bias block (pop next if not first, push prev)
      insn_buf[insn_idx++] = get2DLoadStoreInsn(
          VTA_OPCODE_LOAD,                                    // opcode
          VTA_MEM_ID_ACC,                                     // type
          0,                                                  // sram offset
          (i / VTA_BATCH * out_feat + j) / VTA_BLOCK_OUT,     // dram offset
          block / VTA_BATCH,                                  // y size
          block / VTA_BLOCK_OUT,                              // x size
          //out_feat / VTA_BLOCK_OUT,                           // x stride
          //output_stride / 4 / VTA_BLOCK_OUT,                  // x stride
          output_stride,                                      // x stride in bytes
          0,                                                  // y pad
          0,                                                  // x pad
          0,                                                  // pop prev dep
          (i > 0 || j > 0),                                   // pop next dep
          (virtual_threads == 1),                             // push prev dep
          0);                                                 // push next dep
      // Iterate over input channel blocks
      for (int k = 0; k < in_feat; k += block * virtual_threads) {
        for (int l = 0; l < block * virtual_threads; l += block) {
          // Derive dependence flags
          int pop = (virtual_threads == 1) ?
              1 :
              (i > 0 || j > 0 || k > 0 || l > 0) && (k + l != block * virtual_threads - block);
          int push_prev = (virtual_threads == 1) ?
              ((k + l) != in_feat - block) :
              ((k + l) != in_feat - virtual_threads * block) &&
              (
                  (k + l != in_feat - block) ||
                  (j != out_feat - block) ||
                  (i != batch - block));
          int push_next = (k + l == in_feat - block);
          // Load weight block (pop next)
          insn_buf[insn_idx++] = get2DLoadStoreInsn(
              VTA_OPCODE_LOAD,                                // opcode
              VTA_MEM_ID_WGT,                                 // type
              l / VTA_BLOCK_IN * block / VTA_BLOCK_OUT,       // sram offset
              (j / VTA_BLOCK_OUT * in_feat + k + l) / VTA_BLOCK_IN,  // dram offset
              // (j * in_feat + k + l) / VTA_BLOCK_IN,  // dram offset
              block / VTA_BLOCK_OUT,                          // y size
              block / VTA_BLOCK_IN,                           // x size
              //in_feat / VTA_BLOCK_IN,                         // x stride
              //weight_stride / 4 / VTA_BLOCK_IN,               // x stride
              weight_stride,                                  // x stride in bytes
              0,                                              // y pad
              0,                                              // x pad
              0,                                              // pop prev dep
              pop,                                            // pop next dep
              0,                                              // push prev dep
              0);                                             // push next dep
          // Load input block (push next)
          insn_buf[insn_idx++] = get2DLoadStoreInsn(
              VTA_OPCODE_LOAD,                                // opcode
              VTA_MEM_ID_INP,                                 // type
              l / VTA_BLOCK_IN * block / VTA_BATCH,           // sram offset
              (i / VTA_BATCH * in_feat + k + l) / VTA_BLOCK_IN,  // dram offset
              block / VTA_BATCH,                              // y size
              block / VTA_BLOCK_IN,                           // x size
              //in_feat / VTA_BLOCK_IN,                         // x stride
              //input_stride / 4 / VTA_BLOCK_IN,                // x stride
              input_stride,                                   // x stride
              0,                                              // y pad
              0,                                              // x pad
              0,                                              // pop prev dep
              0,                                              // pop next dep
              0,                                              // push prev dep
              1);                                             // push next dep
          // Perform GEMM (pop prev, push prev if not last, push next if last)
          insn_buf[insn_idx++] = getGEMMInsn(
              l / block * uop_size / virtual_threads,         // uop offset
              block / VTA_BATCH,                              // batch
              block / VTA_BLOCK_IN,                           // in_feat
              block / VTA_BLOCK_OUT,                          // out_feat
              uop_compression,                                // uop_compression
              1,                                              // pop_prev_dep
              0,                                              // pop_next_dep
              push_prev,                                      // push prev dep
              push_next);                                     // push_next_dep
        }
      }
      // Store output block (pop prev, push prev if not last)
      insn_buf[insn_idx++] = get2DLoadStoreInsn(
          VTA_OPCODE_STORE,                                   // opcode
          VTA_MEM_ID_OUT,                                     // type
          0,                                                  // sram offset
          (i / VTA_BATCH * out_feat + j) / VTA_BLOCK_OUT,     // dram offset
          block / VTA_BATCH,                                  // y size
          block / VTA_BLOCK_OUT,                              // x size
          //out_feat / VTA_BLOCK_OUT,                           // x stride
          //out_feat / VTA_BLOCK_OUT * 4,                       // x stride
          //output_stride / VTA_BLOCK_OUT,                      // x stride
          output_stride,                                      // x stride in bytes
          0,                                                  // y pad
          0,                                                  // x pad
          1,                                                  // pop prev dep
          0,                                                  // pop next dep
          1,                                                  // pop prev dep
          0);                                                 // push next dep
    }
  }
  // Finish
  insn_buf[insn_idx++] = getFinishInsn(0, 1);

  return insn_buf;
}

static uint32_t uop_array[8192] BIG_DATA_BSS VTA_DATA;

VTAUop * getGEMMUops(int batch, int in_feat, int out_feat, int uop_compression,
    int multi_threaded) {
  // Derive the total uop size
  int uop_size = (uop_compression) ? batch : batch * in_feat * out_feat;
  if (multi_threaded) uop_size *= 2;

  // Allocate buffer
  // VTAUop *uop_buf = (VTAUop *)(calloc(uop_size, sizeof(VTAUop)));
  VTAUop *uop_buf = (VTAUop *)uop_array;

  if (!uop_compression) {
    int uop_idx = 0;
    for (int i = 0; i < batch; i++) {
      for (int j = 0; j < in_feat; j++) {
        for (int k = 0; k < out_feat; k++) {
          uop_buf[uop_idx].dst_idx = i * out_feat + k;
          uop_buf[uop_idx].src_idx = i * in_feat + j;
          uop_buf[uop_idx].wgt_idx = k * in_feat + j;
          uop_idx++;
        }
      }
    }
  } else {
    for (int i = 0; i < batch; i++) {
      uop_buf[i].dst_idx = i * out_feat;
      uop_buf[i].src_idx = i * in_feat;
      uop_buf[i].wgt_idx = 0;
    }
  }

  if (multi_threaded) {
    if (!uop_compression) {
      int uop_idx = uop_size / 2;
      for (int i = 0; i < batch; i++) {
        for (int j = 0; j < in_feat; j++) {
          for (int k = 0; k < out_feat; k++) {
            uop_buf[uop_idx].dst_idx = i * out_feat + k;
            uop_buf[uop_idx].src_idx = batch * in_feat + i * in_feat + j;
            uop_buf[uop_idx].wgt_idx = out_feat * in_feat + k * in_feat + j;
            uop_idx++;
          }
        }
      }
    } else {
      for (int i = 0; i < batch; i++) {
        uop_buf[batch+i].dst_idx = i * out_feat;
        uop_buf[batch+i].src_idx = batch * in_feat + i * in_feat;
        uop_buf[batch+i].wgt_idx = out_feat * in_feat;
      }
    }
  }

  return uop_buf;
}

void printParameters() {
  // Some debugging code
  printf("Size of VTAInsn: %d\n", sizeof(VTAGenericInsn));
  printf("Size of VTAUop: %d\n", sizeof(VTAUop));
  printf("VTA_UOP_BUFF_DEPTH: %d\n", VTA_UOP_BUFF_DEPTH);
  printf("VTA_LOG_UOP_BUFF_DEPTH: %d\n", VTA_LOG_UOP_BUFF_DEPTH);
  printf("VTA_WGT_BUFF_DEPTH: %d\n", VTA_WGT_BUFF_DEPTH);
  printf("VTA_LOG_WGT_BUFF_DEPTH: %d\n", VTA_LOG_WGT_BUFF_DEPTH);
  printf("VTA_INP_BUFF_DEPTH: %d\n", VTA_INP_BUFF_DEPTH);
  printf("VTA_LOG_INP_BUFF_DEPTH: %d\n", VTA_LOG_INP_BUFF_DEPTH);
  printf("VTA_ACC_BUFF_DEPTH: %d\n", VTA_ACC_BUFF_DEPTH);
  printf("VTA_LOG_ACC_BUFF_DEPTH: %d\n", VTA_LOG_ACC_BUFF_DEPTH);
  printf("VTA_WGT_WORDS: %d\n", VTA_WGT_BUFF_DEPTH*VTA_BLOCK_IN*VTA_BLOCK_OUT);
  printf("VTA_INP_WORDS: %d\n", VTA_INP_BUFF_DEPTH*VTA_BLOCK_IN);
  printf("VTA_ACC_WORDS: %d\n", VTA_ACC_BUFF_DEPTH*VTA_BLOCK_OUT);
  printf("VTA_INS_ELEM_BYTES: %d\n", VTA_INS_ELEM_BYTES);
  printf("VTA_UOP_ELEM_BYTES: %d\n", VTA_UOP_ELEM_BYTES);
  printf("VTA_INP_ELEM_BYTES: %d\n", VTA_INP_ELEM_BYTES);
  printf("VTA_WGT_ELEM_BYTES: %d\n", VTA_WGT_ELEM_BYTES);
  printf("VTA_ACC_ELEM_BYTES: %d\n", VTA_ACC_ELEM_BYTES);
  printf("VTA_BLOCK_IN: %d\n", VTA_BLOCK_IN);
  printf("VTA_BLOCK_OUT: %d\n", VTA_BLOCK_OUT);
}

void printInstruction(int num_insn, VTAGenericInsn *insns) {
  // Keep tabs on dependence queues
  int l2g_queue = 0;
  int g2l_queue = 0;
  int s2g_queue = 0;
  int g2s_queue = 0;
  // Converter
  union VTAInsn c;
  // Iterate over all instructions
  printf("DEBUG - There are %u instructions\n", num_insn);
  for (int i = 0; i < num_insn; i++) {
    // Fetch instruction and decode opcode
    c.generic = insns[i];
    printf("DEBUG - INSTRUCTION %u: ", i);
    if (c.mem.opcode == VTA_OPCODE_LOAD || c.mem.opcode == VTA_OPCODE_STORE) {
      // Print instruction field information
      if (c.mem.opcode == VTA_OPCODE_LOAD) {
        printf("LOAD ");
        if (c.mem.memory_type == VTA_MEM_ID_UOP) printf("UOP\n");
        if (c.mem.memory_type == VTA_MEM_ID_WGT) printf("WGT\n");
        if (c.mem.memory_type == VTA_MEM_ID_INP) printf("INP\n");
        if (c.mem.memory_type == VTA_MEM_ID_ACC) printf("ACC\n");
      }
      if (c.mem.opcode == VTA_OPCODE_STORE) {
        printf("STORE ACC\n");
      }
      printf("\tdep - pop prev: %d, pop next: %d, push prev: %d, push next: %d\n",
             (int)(c.mem.pop_prev_dep),
             (int)(c.mem.pop_next_dep),
             (int)(c.mem.push_prev_dep),
             (int)(c.mem.push_next_dep));
      printf("\tDRAM: 0x%08x, SRAM:0x%04x\n",
             (int)(c.mem.dram_base),
             (int)(c.mem.sram_base));
      printf("\ty: size=%d, pad=[%d, %d]\n",
             (int)(c.mem.y_size),
             (int)(c.mem.y_pad_0),
             (int)(c.mem.y_pad_1));
      printf("\tx: size=%d, stride=%d, pad=[%d, %d]\n",
             (int)(c.mem.x_size),
             (int)(c.mem.x_stride),
             (int)(c.mem.x_pad_0),
             (int)(c.mem.x_pad_1));
      if (c.mem.opcode == VTA_OPCODE_STORE) {
        if (c.mem.pop_prev_dep) g2s_queue--;
        if (c.mem.push_prev_dep) s2g_queue++;
      } else if (c.mem.opcode == VTA_OPCODE_LOAD &&
        (c.mem.memory_type == VTA_MEM_ID_INP || c.mem.memory_type == VTA_MEM_ID_WGT)) {
        if (c.mem.pop_next_dep) g2l_queue--;
        if (c.mem.push_next_dep) l2g_queue++;
      } else {
        if (c.mem.pop_prev_dep) l2g_queue--;
        if (c.mem.push_prev_dep) g2l_queue++;
        if (c.mem.pop_next_dep) s2g_queue--;
        if (c.mem.push_next_dep) g2s_queue++;
      }
    } else if (c.mem.opcode == VTA_OPCODE_GEMM) {
      // Print instruction field information
      printf("GEMM\n");
      printf("\tdep - pop prev: %d, pop next: %d, push prev: %d, push next: %d\n",
             (int)(c.mem.pop_prev_dep),
             (int)(c.mem.pop_next_dep),
             (int)(c.mem.push_prev_dep),
             (int)(c.mem.push_next_dep));
      printf("\trange (%d, %d)\n",
             (int)(c.gemm.uop_bgn),
             (int)(c.gemm.uop_end));
      printf("\treset_out: %d\n", (int)(c.gemm.reset_reg));
      printf("\touter loop - iter: %d, acc: %d, inp: %d, wgt: %d\n",
             (int)(c.gemm.iter_out),
             (int)(c.gemm.dst_factor_out),
             (int)(c.gemm.src_factor_out),
             (int)(c.gemm.wgt_factor_out));
      printf("\tinner loop - iter: %d, acc: %d, inp: %d, wgt: %d\n",
             (int)(c.gemm.iter_in),
             (int)(c.gemm.dst_factor_in),
             (int)(c.gemm.src_factor_in),
             (int)(c.gemm.wgt_factor_in));
      if (c.gemm.pop_prev_dep) l2g_queue--;
      if (c.gemm.push_prev_dep) g2l_queue++;
      if (c.gemm.pop_next_dep) s2g_queue--;
      if (c.gemm.push_next_dep) g2s_queue++;
    } else if (c.mem.opcode == VTA_OPCODE_FINISH) {
      printf("FINISH\n");
      printf("\tdep - pop prev: %d, pop next: %d, push prev: %d, push next: %d\n",
             (int)(c.mem.pop_prev_dep),
             (int)(c.mem.pop_next_dep),
             (int)(c.mem.push_prev_dep),
             (int)(c.mem.push_next_dep));
      if (c.gemm.pop_prev_dep) l2g_queue--;
      if (c.gemm.push_prev_dep) g2l_queue++;
      if (c.gemm.pop_next_dep) s2g_queue--;
      if (c.gemm.push_next_dep) g2s_queue++;
    } 
    else if (c.mem.opcode == VTA_OPCODE_ALU) {
     // Print instruction field information
     printf("ALU - %s\n", getOpcodeString(c.alu.alu_opcode, c.alu.use_imm));
     printf("\tdep - pop prev: %d, pop next: %d, push prev: %d, push next: %d\n",
            (int)(c.mem.pop_prev_dep),
            (int)(c.mem.pop_next_dep),
            (int)(c.mem.push_prev_dep),
            (int)(c.mem.push_next_dep));
     printf("\treset_out: %d\n", (int)(c.alu.reset_reg));
     printf("\trange (%d, %d)\n",
            (int)(c.alu.uop_bgn),
            (int)(c.alu.uop_end));
     printf("\touter loop - iter: %d, dst: %d, src: %d\n",
            (int)(c.alu.iter_out),
            (int)(c.alu.dst_factor_out),
            (int)(c.alu.src_factor_out));
     printf("\tinner loop - iter: %d, dst: %d, src: %d\n",
            (int)(c.alu.iter_in),
            (int)(c.alu.dst_factor_in),
            (int)(c.alu.src_factor_in));
     if (c.alu.pop_prev_dep) l2g_queue--;
     if (c.alu.push_prev_dep) g2l_queue++;
     if (c.alu.pop_next_dep) s2g_queue--;
     if (c.alu.push_next_dep) g2s_queue++;
    }
  }
  printf("DEBUG - l2g_queue = %d, g2l_queue = %d\n", l2g_queue, g2l_queue);
  printf("DEBUG - s2g_queue = %d, g2s_queue = %d\n", s2g_queue, g2s_queue);
}


void printMicroOp(int num_uop, VTAUop *uops) {
  // Iterate over all micro ops
  printf("DEBUG - There are %u micro-ops\n", num_uop);
  for (int i = 0; i < num_uop; i++) {
    // Read micro-op
    printf("DEBUG - UOP %u: ", i);
    printf("acc=%u, inp= %u, wgt=%u\n", uops[i].dst_idx, uops[i].src_idx, uops[i].wgt_idx);
    // if(i==2500) break;
  }
}

const char* getOpcodeString(int opcode, int use_imm) {
  // Returns string name
  if (opcode == VTA_ALU_OPCODE_MIN) {
    if (use_imm) {
      return "min imm";
    } else {
      return "min";
    }
  } else if (opcode == VTA_ALU_OPCODE_MAX) {
    if (use_imm) {
      return "max imm";
    } else {
      return "max";
    }
  } else if (opcode == VTA_ALU_OPCODE_ADD) {
    if (use_imm) {
      return "add imm";
    } else {
      return "add";
    }
  } else if (opcode == VTA_ALU_OPCODE_SHR) {
    return "shr";
  }
  // else if (opcode == VTA_ALU_OPCODE_MUL) {
  //   return "mul";
  // }
  return "unknown op";
}

void printVTADebug(VTAGenericInsn *insn_buf, int ins_size, VTAUop * uop_buf, int uop_size)
{
  printInstruction(ins_size, insn_buf);
  #if 1
  uint32_t *insn_int32;
  insn_int32 = (uint32_t *)insn_buf;
  for(int i=0; i<4*ins_size; i++)
  {
    printf("0x%08x, ", insn_int32[i]);
    if(((i+1)%4)==0) printf("\n");
  }
  printf("\n");
  #endif

  printMicroOp(uop_size, uop_buf);
  #if 1
  uint32_t *uop_buf_in32 = (uint32_t *)uop_buf;
  for(int i=0; i<uop_size; i++)
  {
    printf("0x%08x, ", uop_buf_in32[i]);
    if(((i+1)%4)==0) printf("\n");
    // if(i==100) break;
  }
  printf("\n");
  #endif
}
