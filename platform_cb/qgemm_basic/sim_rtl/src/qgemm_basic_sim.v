`include "ervp_global.vh"
`include "ervp_endian.vh"
`include "ervp_axi_define.vh"
`include "ervp_ahb_define.vh"

`include "ervp_platform_controller_memorymap_offset.vh"
`include "ervp_jtag_memorymap_offset.vh"
`include "ervp_mmiox1_memorymap_offset.vh"

`include "platform_info.vh"
`include "sim_info.vh"
`include "hex_size.vh"

`include "timescale.vh"


module QGEMM_BASIC_SIM
();
///////////
/* clock */
///////////

wire external_clk_0;
wire external_clk_0_pair = 0;

CLOCK_GENERATOR
#(
	.CLK_PERIOD(4)
)
i_gen_external_clk_0
(
	.clk(external_clk_0),
	.rstnn(),
	.rst()
);

wire clk  = i_gen_external_clk_0.clk;
wire rstnn = i_gen_external_clk_0.rstnn;
wire external_rstnn = rstnn;


`include "slow_dram_cell_wire_def.vh"
/////////////
/* signals */
/////////////

wire [`BW_BOOT_MODE-1:0] boot_mode;
wire app_finished = i_platform.i_rtl.platform_controller.app_finished;

wire ncsim_finish_force;
wire ncsim_uart_rx;
wire ncsim_uart_tx;
reg record_enable;
wire simulation_stop;

//////////
// JTAG //
//////////

reg pjtag_rtck;
reg pjtag_rtrstnn;
reg pjtag_rtms;
reg pjtag_rtdi;
wire pjtag_rtdo;
//wire pjtag_rtdo_en = i_platform.i_rtl.platform_controller.pjtag_rtdo_en;
reg pjtag_rfinished;

task wait_jtag_behavior;
begin
  wait(pjtag_rfinished==1);
  #100;
end
endtask

`ifdef INCLUDE_CORE_JTAG

wire core_jtag_tck;
reg core_jtag_trstnn;
wire core_jtag_tms;
wire core_jtag_tdi;
wire core_jtag_tdo;
wire core_jtag_tdo_en;

wire jtag_rflash_clk = 0;
wire [3:0] jtag_rflash_dq_out;
wire [3:0] jtag_rflash_dq_in = 0;
wire [3:0] jtag_rflash_dq_oe;
wire jtag_rflash_cs;

wire rstnn_from_core_jtag;

jtag_ri jtag_rif();
adv_dbg_if_t adv_dbg_if = new(jtag_rif);

initial
begin
	adv_dbg_if.jtag_rreset();
	adv_dbg_if.jtag_rsoftreset();
	adv_dbg_if.init();
	adv_dbg_if.axi4_write32(32'h9401_0000, 1, 32'h1234_5678);
end

initial
begin
	core_jtag_trstnn = 1;
end

always@(*)
begin
	core_jtag_trstnn = jtag_rif.trstn & rstnn;
end

assign core_jtag_tck = jtag_rif.tck;
assign core_jtag_tms = jtag_rif.tms;
assign core_jtag_tdi = jtag_rif.tdi;
assign jtag_rif.tdo = core_jtag_tdo_en? core_jtag_tdo : 1'b 0;

`endif

//////////
// UART //
//////////

`ifdef UART_INDEX_FOR_UART_PRINTF
	wire printf_tx, printf_rx;
`endif

`ifdef INCLUDE_UART_USER
	wire [`NUM_UART_USER-1:0] uart_rx_list;
	wire [`NUM_UART_USER-1:0] uart_tx_list;
`endif

/////////
// SPI //
/////////

genvar iter_spi;

`ifdef INCLUDE_SPI_READYMADE
`ifdef SIMULATE_SPI
  wire [`NUM_SPI_READYMADE-1:0] spi_readymade_sclk_list;
  wire [`NUM_SPI_READYMADE-1:0] spi_readymade_scs_list;
  wire [`NUM_SPI_READYMADE-1:0] spi_readymade_sdq0_list;
  wire [`NUM_SPI_READYMADE-1:0] spi_readymade_sdq1_list;

  generate	
  for(iter_spi=0; iter_spi<`NUM_SPI_READYMADE; iter_spi=iter_spi+1)
  begin : i_gen_spi_readymade
    VIRTUAL_SPI_SLAVE
    i_virtual_spi
    (
      .csn   (spi_readymade_scs_list[iter_spi]),
      .sck   (spi_readymade_sclk_list[iter_spi]),
      .din   (spi_readymade_sdq0_list[iter_spi]),
      .dout  (spi_readymade_sdq1_list[iter_spi])
    );
    assign spi_readymade_sclk_list = i_platform.spi_readymade_sclk_list;
    assign spi_readymade_scs_list = i_platform.spi_readymade_scs_list;
    assign spi_readymade_sdq0_list = i_platform.spi_readymade_sdq0_list;
    assign i_platform.spi_readymade_sdq1_list = spi_readymade_sdq1_list;
  end
  endgenerate

`endif
`endif

/*

`ifdef USE_SPI_FLASH

	wire spi_sflash_clk;
	wire spi_sflash_cs;
	wire [3:0] spi_sflash_dq;

`ifndef DISABLE_SPI_FLASH
	assign spi_sflash_clk = spi_sclk[`SPI_INDEX_FOR_SPI_FLASH];
	assign spi_sflash_cs = spi_scs[`SPI_INDEX_FOR_SPI_FLASH];

	TRI_STATE_BUFFER
	i_spi_flash_dq0
	(
		.output_enable(spi_sdq0_oe[`SPI_INDEX_FOR_SPI_FLASH]),
		.in_value(spi_sdq0_out[`SPI_INDEX_FOR_SPI_FLASH]),
		.out_value(spi_sdq0_in[`SPI_INDEX_FOR_SPI_FLASH]),
		.tri_value(spi_sflash_dq[0])
	);

	TRI_STATE_BUFFER
	i_spi_flash_dq1
	(
		.output_enable(spi_sdq1_oe[`SPI_INDEX_FOR_SPI_FLASH]),
		.in_value(spi_sdq1_out[`SPI_INDEX_FOR_SPI_FLASH]),
		.out_value(spi_sdq1_in[`SPI_INDEX_FOR_SPI_FLASH]),
		.tri_value(spi_sflash_dq[1])
	);

	TRI_STATE_BUFFER
	i_spi_flash_dq2
	(
		.output_enable(spi_sdq2_oe[`SPI_INDEX_FOR_SPI_FLASH]),
		.in_value(spi_sdq2_out[`SPI_INDEX_FOR_SPI_FLASH]),
		.out_value(spi_sdq2_in[`SPI_INDEX_FOR_SPI_FLASH]),
		.tri_value(spi_sflash_dq[2])
	);

	TRI_STATE_BUFFER
	i_spi_flash_dq3
	(
		.output_enable(spi_sdq3_oe[`SPI_INDEX_FOR_SPI_FLASH]),
		.in_value(spi_sdq3_out[`SPI_INDEX_FOR_SPI_FLASH]),
		.out_value(spi_sdq3_in[`SPI_INDEX_FOR_SPI_FLASH]),
		.tri_value(spi_sflash_dq[3])
	);

	N25Qxxx i_flash_model
	(
		.S(spi_sflash_cs), 
		.C_(spi_sflash_clk), 
		.DQ0(spi_sflash_dq[0]), 
		.DQ1(spi_sflash_dq[1]),
		.Vpp_W_DQ2(spi_sflash_dq[2]),
		.HOLD_DQ3(spi_sflash_dq[3]),
		.Vcc(32'h00000bb8) 
	);
`endif
`endif
*/

/////////
// I2C //
/////////

`ifdef INCLUDE_I2C_READYMADE
`ifdef SIMULATE_I2C

	genvar iter_i2c;

	generate
	for(iter_i2c=0; iter_i2c<`NUM_I2C_READYMADE; iter_i2c=iter_i2c+1)
	begin : i_gen_i2c_readymade
		
    pullup i_pullup_i2c_clk (i_platform.i2c_readymade_sclk_list[iter_i2c]);
    pullup i_pullup_i2c_data (i_platform.i2c_readymade_sdata_list[iter_i2c]);

		I2C_SLAVE_MODEL
		#(
			.I2C_ADR(7'h43+iter_i2c)
		)
		i_virtual_i2c
		(
			.scl(i_platform.i2c_readymade_sclk_list[iter_i2c]),
			.sda(i_platform.i2c_readymade_sdata_list[iter_i2c])
		);
	end
	endgenerate

`endif
`endif

//////////
// GPIO //
//////////

`ifdef INCLUDE_GPIO_USER
	wire [`NUM_GPIO_USER-1:0] gpio_list;
`endif

///////////////
// SPI FLASH //
///////////////

`ifdef INCLUDE_SPI_FLASH
`ifdef SIMULATE_SPI_FLASH

  wire spi_flash_sdq2;
  wire spi_flash_sdq3;

  pullup i_pullup_dq0 (i_platform.spi_flash_sdq0);
  pullup i_pullup_dq1 (i_platform.spi_flash_sdq1);
  pullup i_pullup_dq2 (spi_flash_sdq2);
  pullup i_pullup_dq3 (spi_flash_sdq3);

	N25Qxxx i_flash_model
	(
		.S(i_platform.spi_flash_scs), 
		.C_(i_platform.spi_flash_sclk), 
		.DQ0(i_platform.spi_flash_sdq0), 
		.DQ1(i_platform.spi_flash_sdq1),
		.Vpp_W_DQ2(spi_flash_sdq2),
		.HOLD_DQ3(spi_flash_sdq3),
		.Vcc(32'h00000bb8) 
	);

`endif
`endif

//////////
// WIFI //
//////////

`ifdef INCLUDE_WIFI

reg wifi_sitr;

assign i_platform.wifi_sitr = wifi_sitr;

initial
begin
	wifi_sitr = 1;
	#1000000;
	wifi_sitr = 0;
end

`endif

/////////
// C2C //
/////////

`ifdef SIMULATE_C2C_SELF

assign i_platform.c2cs00_csn = i_platform.c2cm_csn[0];
assign i_platform.c2cs00_clk = i_platform.c2cm_clk;
assign i_platform.tristate_buffer_for_c2cs00_dq_rival_list = i_platform.tristate_buffer_for_c2cm_dq_roval_list;
assign i_platform.tristate_buffer_for_c2cm_dq_rival_list = i_platform.tristate_buffer_for_c2cs00_dq_roval_list;

`endif

/////////
// I2S //
/////////

/*
`ifdef INCLUDE_I2S

reg i_i2s_slave_wrequest;
reg i_i2s_slave_rrequest;

initial
begin
	i_i2s_slave_wrequest = 0;
	wait(i_platform.i2s_tx_lrck==0);
	wait(i_platform.i2s_tx_lrck==1);
	i_i2s_slave_wrequest = 1;
end

initial
begin
	i_i2s_slave_rrequest = 0;
	wait(i_platform.i2s_rx_lrck==0);
	wait(i_platform.i2s_rx_lrck==1);
	i_i2s_slave_rrequest = 1;
end

ERVP_FIFO
#(
	.BW_DATA(1),
	.DEPTH(128)
)
i_i2s_slave
(
	.clk(i_platform.i2s_tx_mclk | i_platform.i2s_rx_mclk),
	.rstnn(rstnn),
	.enable(1'b 1),
	.wready(),
	.wrequest(i_i2s_slave_wrequest),
	.wfull(),
	.wdata(i_platform.i2s_tx_dout),
	.rready(),
	.rrequest(i_i2s_slave_rrequest),
	.rempty(),
	.rdata(i_platform.i2s_rx_din)
);

`endif
*/

//////////
// HBC1 //
//////////

`ifdef INCLUDE_HBC1_TX
	`ifdef INCLUDE_HBC1_RX
		wire hbc1_tx_clk;
		wire hbc1_tx_data;
		wire hbc1_rx_clk;
		wire hbc1_rx_data;

		/*
		ERVP_ASYNCH_FIFO
		#(
			.BW_DATA(1),
			.DEPTH(16)
		)
		i_human_body
		(
			.wclk(hbc1_tx_clk),
			.wrstnn(external_rstnn),
			.wready(),
			.wrequest(1'b 1),
			.wfull(),
			.wdata(hbc1_tx_data),
			.rclk(hbc1_rx_clk),
			.rrstnn(external_rstnn),
			.rready(),
			.rrequest(1'b 1),
			.rempty(),
			.rdata(hbc1_rx_data)
		);
		*/

		assign hbc1_rx_data = hbc1_tx_data;

		assign hbc1_tx_clk = i_platform.i_rtl.clk_hbc1_tx;
		assign hbc1_tx_data = i_platform.hbc1_tx_data;

		assign hbc1_rx_clk = i_platform.i_rtl.clk_hbc1_rx;
		assign i_platform.hbc1_rx_data = hbc1_rx_data;
	`endif
`endif

///////////
// SDRAM //
///////////

`ifdef SIMULATE_SDRAM_BEHAVIOR

localparam BW_SDRAM_DATA = 32;

genvar i_sdram_dq;

wire [BW_SDRAM_DATA-1:0]    LPSDR_DQ;
wire            SCLK2SDR;
wire            nSSCCS02SDR;
wire            nSSCCS12SDR;
wire            nSSCRAS2SDR;
wire            nSSCCAS2SDR;
wire            nSSCWEX2SDR;
wire    [12:0]  SSCADDR2SDR;
wire    [1:0]   SSCBADR2SDR;
wire    [3:0]   SSCDQM2SDR;
wire    [3:0]   nSSCDOE2SDR;

assign i_platform.LPSDR_CLK_FB = i_platform.LPSDR_CLK;

generate
for(i_sdram_dq=0; i_sdram_dq < BW_SDRAM_DATA; i_sdram_dq = i_sdram_dq + 1)
begin : i_connect_sdram_dq
	assign LPSDR_DQ[i_sdram_dq] = i_platform.tristate_buffer_for_LPSDR_DQ_rod_list[i_sdram_dq]? 1'hz : i_platform.LPSDR_DQ[i_sdram_dq];
	assign i_platform.LPSDR_DQ[i_sdram_dq] = i_platform.tristate_buffer_for_LPSDR_DQ_rod_list[i_sdram_dq]? LPSDR_DQ[i_sdram_dq] : 1'hz;
end
endgenerate

// PAD delay

assign SCLK2SDR    = i_platform.clk_sdram_cell;
assign nSSCCS02SDR = LPSDR_CS0_N;
assign nSSCCS12SDR = LPSDR_CS1_N;
assign nSSCRAS2SDR = LPSDR_RAS_N;
assign nSSCCAS2SDR = LPSDR_CAS_N;
assign nSSCWEX2SDR = LPSDR_WE_N;
assign SSCADDR2SDR = LPSDR_A;
assign SSCBADR2SDR = LPSDR_BA;
assign SSCDQM2SDR  = LPSDR_DQM;

// cell

IS42VM16160K uSDR_L (
    .dq                 ( LPSDR_DQ[15:0]     ),
    .addr               ( SSCADDR2SDR       ),
    .ba                 ( SSCBADR2SDR       ),
    .clk                ( SCLK2SDR          ),
    .cke                ( 1'b1              ),
    .csb                ( nSSCCS02SDR       ),
    .rasb               ( nSSCRAS2SDR       ),
    .casb               ( nSSCCAS2SDR       ),
    .web                ( nSSCWEX2SDR       ),
    .ldqm               ( SSCDQM2SDR[0]     ),
    .udqm               ( SSCDQM2SDR[1]     )
);

IS42VM16160K uSDR_U (
    .dq                 ( LPSDR_DQ[31:16]    ),
    .addr               ( SSCADDR2SDR       ),
    .ba                 ( SSCBADR2SDR       ),
    .clk                ( SCLK2SDR          ),
    .cke                ( 1'b1              ),
    .csb                ( nSSCCS02SDR       ),
    .rasb               ( nSSCRAS2SDR       ),
    .casb               ( nSSCCAS2SDR       ),
    .web                ( nSSCWEX2SDR       ),
    .ldqm               ( SSCDQM2SDR[2]     ),
    .udqm               ( SSCDQM2SDR[3]     )
);

`endif

//////////////
// EXT_MRAM //
//////////////

`ifdef SIMULATE_EXT_MRAM_BEHAVIOR

localparam BW_EXT_MRAM_DATA = 32;

wire EXTMR_E_N;
wire EXTMR_W_N;
wire EXTMR_G_N;
wire [(4)-1:0] EXTMR_BE_N;
wire [(21)-1:0] EXTMR_A;
wire [BW_EXT_MRAM_DATA-1:0]    EXTMR_DQ;

wire [(32)-1:0] EXTMR_DQ_sod;
wire [(32)-1:0] EXTMR_DQ_soval;
wire [(32)-1:0] EXTMR_DQ_sival;

genvar i_mram_dq;

generate
for(i_mram_dq=0; i_mram_dq < BW_EXT_MRAM_DATA; i_mram_dq = i_mram_dq + 1)
begin : i_connect_mram_dq
	assign EXTMR_DQ[i_mram_dq] = i_platform.tristate_buffer_for_EXTMR_DQ_rod_list[i_mram_dq]? 1'hz : i_platform.EXTMR_DQ[i_mram_dq];
	assign i_platform.EXTMR_DQ[i_mram_dq] = i_platform.tristate_buffer_for_EXTMR_DQ_rod_list[i_mram_dq]? EXTMR_DQ[i_mram_dq] : 1'hz;
end
endgenerate

assign EXTMR_E_N = i_platform.EXTMR_E_N;
assign EXTMR_W_N = i_platform.EXTMR_W_N;
assign EXTMR_G_N = i_platform.EXTMR_G_N;
assign EXTMR_BE_N = i_platform.EXTMR_BE_N;
assign #1 EXTMR_A = i_platform.EXTMR_A;

MR5A16A i_mram_up
(
  .Address(EXTMR_A), 
  .DQ(EXTMR_DQ[31:16]), 
  .G_b(EXTMR_G_N), 
  .E_b(EXTMR_E_N), 
  .W_b(EXTMR_W_N),
  .UB_b(EXTMR_BE_N[3]), 
  .LB_b(EXTMR_BE_N[2])
);

MR5A16A i_mram_low
(
  .Address(EXTMR_A),
  .DQ(EXTMR_DQ[15:0]),
  .G_b(EXTMR_G_N),
  .E_b(EXTMR_E_N),
  .W_b(EXTMR_W_N),
  .UB_b(EXTMR_BE_N[1]),
  .LB_b(EXTMR_BE_N[0])
);

`endif

///////////
/* ncsim */
///////////

assign ncsim_finish_force = 0;

`ifdef INCLUDE_UART_PRINTF
	assign ncsim_uart_rx = printf_tx;
	assign printf_rx = ncsim_uart_tx;
`else
	assign ncsim_uart_rx = 0;
`endif

NCSIM_MANAGER
#(
	.UART_CLK_HZ(`UART_CLK_HZ)
)
i_ncsim_manager
(
  .finish_force(ncsim_finish_force),
  .uart_rx(ncsim_uart_rx),
	.uart_tx(ncsim_uart_tx),
  .record_enable(record_enable),
  .simulation_stop(simulation_stop)
);

`include "ncsim_record.vb"

QGEMM_BASIC
i_platform
(
	.external_rstnn(external_rstnn),
	.pjtag_rtck(pjtag_rtck),
	.pjtag_rtrstnn(pjtag_rtrstnn),
	.pjtag_rtms(pjtag_rtms),
	.pjtag_rtdi(pjtag_rtdi),
	.pjtag_rtdo(pjtag_rtdo),
	.external_clk_0(external_clk_0),
	.external_clk_0_pair(external_clk_0_pair),
	.printf_tx(printf_tx),
	.printf_rx(printf_rx)
	`include "slow_dram_cell_port_mapping.vh"
);

//////////
/* jtag */
//////////

assign boot_mode = `BOOT_MODE_OCD;

`include "ervp_endian.vf"
`include "ervp_jtag_util.vf"
`include "ervp_platform_controller_util.vf"

`ifndef SRAM_HEX_FILE
`define SRAM_HEX_FILE "sram.hex"
`endif

`ifndef SRAM_OFFSET
`define SRAM_OFFSET 32'h 0
`endif

`ifndef CRM_HEX_FILE
`define CRM_HEX_FILE "crm.hex"
`endif

`ifndef CRM_OFFSET
`define CRM_OFFSET 32'h 0
`endif

`ifndef DRAM_HEX_FILE
`define DRAM_HEX_FILE "dram.hex"
`endif

`ifndef DRAM_OFFSET
`define DRAM_OFFSET 32'h 0
`endif

`ifndef MEMORY_ENDIAN
`define MEMORY_ENDIAN `LITTLE_ENDIAN
`endif

`ifdef FAST_APP_LOAD
`ifdef USE_LARGE_RAM
`ifndef SIMULATE_SDRAM_BEHAVIOR
`define FAST_APP_LOAD_LARGE_RAM
`endif
`endif // USE_LARGE_RAM
`endif

`ifdef FAST_APP_LOAD_LARGE_RAM
`define VLRAM_CELL i_platform.`DRAM_IP_INSTANCE.i_cell.memory
`define MRAM_UP_CELL i_mram_up.MRAM_Array
`define MRAM_LOW_CELL i_mram_low.MRAM_Array
`endif

`define BW_WORD 32
`define SRAM_CELL_SIZE_IN_WORD (`SRAM_CELL_SIZE/(`BW_WORD/`BW_BYTE))

integer reset_cycle;
reg first_reset_is_complete;

reg app_load_request;
reg app_is_loaded;

integer i,j;
integer word_index;
integer cell_index;
integer word_index_in_cell;
integer word_index_in_line;
integer line_index_in_cell;
integer direct_memory_load;
integer addr;
integer num_word_in_line;
integer ddr_file;
integer error;

reg [32-1:0] hex_memory [`HEX_SIZE-1:0];

initial
begin
	pjtag_rfinished = 0;
	app_load_request = 0;
	#1
	wait(rstnn==0);
	wait(rstnn==1);
	if(boot_mode==`BOOT_MODE_STAND_ALONE)
	begin
		$display("[JTAG:INFO] do NOTHING");
	end
	else if(boot_mode==`BOOT_MODE_OCD)
	begin
		wait(first_reset_is_complete==1);
		init_jtag();
		release_all_reset();
		app_load_request = 1;
		wait(app_is_loaded==1);
    set_app_addr();
		set_sim_env();
		set_all_ready();
	end
	else
	begin
		$display("[JTAG:ERROR] undefined boot mode");
		$finish;
	end
	#1
	$display("[JTAG:INFO] finished");
	pjtag_rfinished = 1;
end

initial
begin
	first_reset_is_complete = 0;
	reset_cycle = 0;
	#1
	wait(rstnn==0);
	wait(rstnn==1);
	while(1)
	begin
		wait(clk==0);
		wait(clk==1);
		if(reset_cycle==100*20)
			break;
		else
			reset_cycle = reset_cycle + 1;
	end
	first_reset_is_complete = 1;
end

initial
begin
  `ifdef USE_SMALL_RAM
    if((`SRAM_HEX_SIZE*4) > `SIM_SMALL_RAM_SIZE)
    begin
      $display("[Error] SIM_SMALL_RAM_SIZE is too small");
      $finish;
    end
  `endif
  `ifdef USE_LARGE_RAM
    if((`DRAM_HEX_SIZE*4) > `SIM_LARGE_RAM_SIZE)
    begin
      $display("[Error] SIM_LARGE_RAM_SIZE is too small");
      $finish;
    end
  `endif
  `ifndef FAST_APP_LOAD
    $display("[Warning] Please define BUILD_MODE=RELEASE");
    $display("[Warning] in rvx_each.mh or ../user/sim/env/set_sim_env.mh");
    $display("[Warning] Or simulation takes too long\n");
  `endif
end

initial
begin
	app_is_loaded = 0;
	direct_memory_load = 0;
	#1
	wait(rstnn==0);
	wait(rstnn==1);
	if(boot_mode==`BOOT_MODE_OCD)
	begin
		wait(app_load_request==1);
		#1
		// sram
		`ifdef USE_SMALL_RAM
      $readmemh(`SRAM_HEX_FILE, hex_memory);
      addr = `SMALL_RAM_BASEADDR;
      `ifdef FAST_APP_LOAD
        direct_memory_load = 1;
        num_word_in_line = `SRAM_CELL_VARIABLE_WIDTH/`BW_WORD;
        $display("[JTAG:INFO] SRAM fast load start");
        for(i=0; i<`SRAM_HEX_SIZE; i=i+1)
        begin
          word_index = `SRAM_OFFSET + i;
          cell_index = word_index / `SRAM_CELL_SIZE_IN_WORD;
          word_index_in_cell = word_index % `SRAM_CELL_SIZE_IN_WORD;
          line_index_in_cell = word_index_in_cell / num_word_in_line;
          word_index_in_line = word_index_in_cell % num_word_in_line;

  				`define SRAM_CELL0_VARIABLE i_platform.`SRAM_IP_INSTANCE.generate_cell[0].`SRAM_CELL_VARIABLE_INSIDE_MEMORY_IP

          case(cell_index)
            0:
            begin
              `SRAM_CELL0_VARIABLE[line_index_in_cell][(word_index_in_line+1)*`BW_WORD-1-:`BW_WORD] = CHANGE_ENDIAN_HEX2MAN(32,`MEMORY_ENDIAN,hex_memory[i]);
            end

          endcase
        end
        direct_memory_load = 0;
      `else
        $display("[JTAG:INFO] SRAM slow load start %d", `SRAM_HEX_SIZE);
        write_memory_using_jtag(addr, CHANGE_ENDIAN_HEX2MAN(32,`MEMORY_ENDIAN,hex_memory[0]));
        print_memory_using_jtag(addr);
        for(i=0; i<`SRAM_HEX_SIZE; i=i+1)
        begin
          if((i&32'h FF)==32'h FF)
          begin
            $display("[JTAG:INFO] SRAM slow load is processing... %8d", i);
          end
          write_memory_using_jtag(addr, CHANGE_ENDIAN_HEX2MAN(32,`MEMORY_ENDIAN,hex_memory[i]));
          addr = addr + 4;
        end
      `endif
      $display("[JTAG:INFO] SRAM load end");
    `endif

		// dram
    `ifdef INCLUDE_EXT_MRAM
      $display("[JTAG:INFO] MRAM config set");
      write_memory_using_jtag(`I_SYSTEM_EXT_MRAM_CONTROL_BASEADDR+`MMAP_OFFSET_MMIO_CORE_CONFIG_SAWD, (1<<3));
    `endif
		`ifdef USE_LARGE_RAM
			$readmemh(`DRAM_HEX_FILE, hex_memory);
			addr = `LARGE_RAM_BASEADDR;
			num_word_in_line = `DRAM_WIDTH/32;
			`ifdef FAST_APP_LOAD_LARGE_RAM
				direct_memory_load = 1;
        `ifdef SIMULATE_EXT_MRAM_BEHAVIOR
  				$display("[JTAG:INFO] MRAM fast load start");
          for(i=0; i<`DRAM_HEX_SIZE; i=i+1)
          begin
            word_index = `DRAM_OFFSET + i;
            word_index_in_line = word_index % num_word_in_line;
            line_index_in_cell = word_index / num_word_in_line;
          {`MRAM_UP_CELL[line_index_in_cell][(word_index_in_line+1)*16-1-:16],`MRAM_LOW_CELL[line_index_in_cell][(word_index_in_line+1)*16-1-:16]} = CHANGE_ENDIAN_HEX2MAN(32,`MEMORY_ENDIAN,hex_memory[i]);
            //$display("%8x : %8x", word_index, hex_memory[i]);
          end
        `else
          $display("[JTAG:INFO] Virtual RAM fast load start");
          for(i=0; i<`DRAM_HEX_SIZE; i=i+1)
          begin
            word_index = `DRAM_OFFSET + i;
            word_index_in_line = word_index % num_word_in_line;
            line_index_in_cell = word_index / num_word_in_line;
            `VLRAM_CELL[line_index_in_cell][(word_index_in_line+1)*32-1-:32] = CHANGE_ENDIAN_HEX2MAN(32,`MEMORY_ENDIAN,hex_memory[i]);
            //$display("%8x : %8x", word_index, hex_memory[i]);
          end
        `endif
				direct_memory_load = 0;
			`else
        `ifdef SIMULATE_EXT_MRAM_BEHAVIOR
  				$display("[JTAG:INFO] MRAM slow load start %d", `DRAM_HEX_SIZE);
        `else
          $display("[JTAG:INFO] Virtual RAM slow load start %d", `DRAM_HEX_SIZE);
        `endif
				for(i=0; i<`DRAM_HEX_SIZE; i=i+1)
				begin
          if((i&32'h FF)==32'h FF)
          begin
            `ifdef SIMULATE_EXT_MRAM_BEHAVIOR
              $display("[JTAG:INFO] MRAM slow load is processing... %8d", i);
            `else
              $display("[JTAG:INFO] Virtual RAM slow load is processing... %8d", i);
            `endif
          end
					write_memory_using_jtag(addr, CHANGE_ENDIAN_HEX2MAN(32,`MEMORY_ENDIAN,hex_memory[i]));
					addr = addr + 4;
				end
			`endif
      `ifdef SIMULATE_EXT_MRAM_BEHAVIOR
        $display("[JTAG:INFO] MRAM load end");
      `else
        $display("[JTAG:INFO] Virtual RAM load end");
      `endif
		`endif
		#1
		app_is_loaded = 1;
	end
end


endmodule