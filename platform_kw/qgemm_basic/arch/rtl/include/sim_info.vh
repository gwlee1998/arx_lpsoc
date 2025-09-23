`ifndef __SIM_INFO_H__
`define __SIM_INFO_H__


//`define SIMULATE_SPI
//`define SIMULATE_I2C
//`define SIMULATE_SPI_FLASH
//`define SIMULATE_SDRAM_BEHAVIOR
//`define SIMULATE_EXT_MRAM_BEHAVIOR
//`define SIMULATE_C2C_SELF
`define SIM_SMALL_RAM_SIZE 131072
`define SIM_LARGE_RAM_SIZE 2097152
`define RECORD_TOP "depth=20",i_platform.i_rtl.i_dca_matrix_qgemm
`define SRAM_IP_INSTANCE i_system_sram
`define SRAM_CELL_VARIABLE_INSIDE_MEMORY_IP i_cell.memory
`define SRAM_CELL_VARIABLE_WIDTH 32
`define NUM_SRAM_CELL 1
`define DRAM_IP_INSTANCE i_system_ddr
`define DRAM_WIDTH 32
`define EXTERNAL_CLK_NAME_00 external_clk_0

`endif