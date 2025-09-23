#ifndef __VTA_CTRL_API_H__
#define __VTA_CTRL_API_H__

#include "ervp_mmio_util.h"

#include "vta_ervp.h"
#include "vta_hw_spec.h"

int vta_status(const vta_hwinfo_t *const hwinfo);
void vta_ctrl_wait(const vta_hwinfo_t *const hwinfo);
void vta_ctrl_write(const vta_hwinfo_t *const hwinfo, unsigned int value);
void vta_setup_register_all(const vta_hwinfo_t *const hwinfo, int ins_size, VTAGenericInsn *insns, VTAUop *uops, void *input, void *weight, void *bias, void *output);
void vta_setup_register_bias(const vta_hwinfo_t *const hwinfo, void *bias);

#endif