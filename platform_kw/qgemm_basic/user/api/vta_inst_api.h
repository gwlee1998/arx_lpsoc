#ifndef __VTA_INST_API_H__
#define __VTA_INST_API_H__

#include "vta_hw_spec.h"

VTAGenericInsn get1DLoadStoreInsn(int opcode, int type, int sram_offset, int dram_offset, int size,
    int pop_prev_dep, int pop_next_dep, int push_prev_dep, int push_next_dep);
VTAGenericInsn get2DLoadStoreInsn(int opcode, int type, int sram_offset, int dram_offset,
    int y_size, int x_size, int x_stride, int y_pad, int x_pad, int pop_prev_dep, int pop_next_dep,
    int push_prev_dep, int push_next_dep);

VTAGenericInsn getGEMMInsn(int uop_offset, int batch, int in_feat, int out_feat,
    int uop_compression, int pop_prev_dep, int pop_next_dep, int push_prev_dep,
    int push_next_dep);
VTAGenericInsn *getVTAGEMMInsn(int batch, int in_channels, int out_channels, int uop_compression);
VTAGenericInsn *getVTABlockedGEMMInsn(int batch, int in_channels, int out_channels, int block, int uop_compression, int virtual_threads);
VTAUop * getGEMMUops(int batch, int in_feat, int out_feat, int uop_compression, int multi_threaded);

VTAGenericInsn getALUInsn(int opcode, int vector_size, int use_imm, int imm, int uop_compression,
    int pop_prev_dep, int pop_next_dep, int push_prev_dep, int push_next_dep);
VTAGenericInsn *getVTAALUInsn(int opcode, int batch, int vector_size, int uop_compression);
VTAUop * getMapALUUops(int vector_size, int uop_compression);

VTAGenericInsn getFinishInsn(int pop_prev, int pop_next);

void printParameters();
void printInstruction(int num_insn, VTAGenericInsn *insns);
void printMicroOp(int num_uop, VTAUop *uops);
const char* getOpcodeString(int opcode, int use_imm);

void printVTADebug(VTAGenericInsn *insn_buf, int ins_size, VTAUop * uop_buf, int uop_size);

VTAGenericInsn *getVTABlockedGEMMInsn_block16_stride(int input_stride, int weight_stride, int output_stride);

#endif
