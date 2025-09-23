#include <stdint.h>

#include "ervp_printf.h"
#include "vta_ctrl_api.h"

// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// control
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - enable ap_done interrupt (Read/Write)
//        bit 1  - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - ap_done (COR/TOW)
//        bit 1  - ap_ready (COR/TOW)
//        others - reserved
// 0x10 : Data signal of insn_count
//        bit 31~0 - insn_count[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of insns
//        bit 31~0 - insns[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of uops
//        bit 31~0 - uops[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of inputs
//        bit 31~0 - inputs[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of weights
//        bit 31~0 - weights[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of biases
//        bit 31~0 - biases[31:0] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of outputs
//        bit 31~0 - outputs[31:0] (Read/Write)
// 0x44 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XVTA_CONTROL_ADDR_AP_CTRL(x) ((x) + 0x00)
// #define XVTA_CONTROL_ADDR_GIE             (I_AXI_HOST_SLAVE_BASEADDR + 0x04)
// #define XVTA_CONTROL_ADDR_IER             (I_AXI_HOST_SLAVE_BASEADDR + 0x08)
// #define XVTA_CONTROL_ADDR_ISR             (I_AXI_HOST_SLAVE_BASEADDR + 0x0c)
#define XVTA_CONTROL_ADDR_INSN_COUNT_DATA(x) ((x) + 0x08)
#define XVTA_CONTROL_BITS_INSN_COUNT_DATA 32
#define XVTA_CONTROL_ADDR_INSNS_DATA(x) ((x) + 0x0c)
#define XVTA_CONTROL_BITS_INSNS_DATA 32
#define XVTA_CONTROL_ADDR_UOPS_DATA(x) ((x) + 0x10)
#define XVTA_CONTROL_BITS_UOPS_DATA 64
#define XVTA_CONTROL_ADDR_INPUTS_DATA(x) ((x) + 0x14)
#define XVTA_CONTROL_BITS_INPUTS_DATA 64
#define XVTA_CONTROL_ADDR_WEIGHTS_DATA(x) ((x) + 0x18)
#define XVTA_CONTROL_BITS_WEIGHTS_DATA 64
#define XVTA_CONTROL_ADDR_BIASES_DATA(x) ((x) + 0x1c)
#define XVTA_CONTROL_BITS_BIASES_DATA 64
#define XVTA_CONTROL_ADDR_OUTPUTS_DATA(x) ((x) + 0x20)
#define XVTA_CONTROL_BITS_OUTPUTS_DATA 64

int vta_status(const vta_hwinfo_t *const hwinfo)
{
  return mmio_read_data(XVTA_CONTROL_ADDR_AP_CTRL(hwinfo->ctrl_addr));
}

void vta_ctrl_wait(const vta_hwinfo_t *const hwinfo)
{
  while (1)
  {
    int status = vta_status(hwinfo);
    if (status == 0)
      break;
    if ((status & 0x2) != 0)
      break;
  }
}

void vta_ctrl_write(const vta_hwinfo_t *const hwinfo, unsigned int value)
{
  mmio_write_data(XVTA_CONTROL_ADDR_AP_CTRL(hwinfo->ctrl_addr), value);
}

void vta_setup_register_all(const vta_hwinfo_t *const hwinfo, int ins_size, VTAGenericInsn *insns, VTAUop *uops,
                        void *input, void *weight, void *bias, void *output)
{
  // Data signal of insn_count
  mmio_write_data(XVTA_CONTROL_ADDR_INSN_COUNT_DATA(hwinfo->ctrl_addr), (mmio_data_t)ins_size);
  // Data signal of insns
  mmio_write_data(XVTA_CONTROL_ADDR_INSNS_DATA(hwinfo->ctrl_addr), (mmio_data_t)insns);
  // Data signal of uops
  mmio_write_data(XVTA_CONTROL_ADDR_UOPS_DATA(hwinfo->ctrl_addr), (mmio_data_t)uops);
  // Data signal of inputs
  mmio_write_data(XVTA_CONTROL_ADDR_INPUTS_DATA(hwinfo->ctrl_addr), (mmio_data_t)input);
  // Data signal of weights
  mmio_write_data(XVTA_CONTROL_ADDR_WEIGHTS_DATA(hwinfo->ctrl_addr), (mmio_data_t)weight);
  // Data signal of biases
  mmio_write_data(XVTA_CONTROL_ADDR_BIASES_DATA(hwinfo->ctrl_addr), (mmio_data_t)bias);
  // Data signal of outputs
  mmio_write_data(XVTA_CONTROL_ADDR_OUTPUTS_DATA(hwinfo->ctrl_addr), (mmio_data_t)output);
}

void vta_setup_register_bias(const vta_hwinfo_t *const hwinfo, void *bias)
{
  mmio_write_data(XVTA_CONTROL_ADDR_BIASES_DATA(hwinfo->ctrl_addr), (mmio_data_t)bias);
}