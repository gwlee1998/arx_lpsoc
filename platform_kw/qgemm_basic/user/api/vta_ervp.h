#ifndef __VTA_ERVP_H__
#define __VTA_ERVP_H__

#include "ervp_mmio_util.h"
#include "ervp_task.h"
#include "ervp_variable_allocation.h"
#include "platform_info.h"

#define VTA_DATA __attribute__ ((aligned(0x1000)))
#define VTA_CTRL __attribute__ ((aligned(0x1000)))

typedef struct
{
  int block_size;
  mmio_addr_t ctrl_addr;
  ervp_task_wait_fx_t wait_fx;
} vta_hwinfo_t;

#endif