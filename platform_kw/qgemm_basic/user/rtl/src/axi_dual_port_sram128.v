
`include "timescale.vh"
`include "ervp_global.vh"
`include "ervp_axi_define.vh"
`include "munoc_network_include.vh"
module axi_dual_port_sram128
(
	clk,
	rstnn,

	rxawid,
	rxawaddr,
	rxawlen,
	rxawsize,
	rxawburst,
	rxawvalid,
	rxawready,

	rxwid,
	rxwdata,
	rxwstrb,
	rxwlast,
	rxwvalid,
	rxwready,

	rxbid,
	rxbresp,
	rxbvalid,
	rxbready,

	rxarid,
	rxaraddr,
	rxarlen,
	rxarsize,
	rxarburst,
	rxarvalid,
	rxarready,

	rxrid,
	rxrdata,
	rxrresp,
	rxrlast,
	rxrvalid,
	rxrready,

	int_renable_list,
	int_rdata_list,
	int_rvalid_list,
	int_index_list,
	int_wenable_list,
	int_byte_enable_list,
	int_wdata_list
);

////////////////////////////
/* parameter input output */
////////////////////////////

localparam CAPACITY = 32768;  // in bytes
localparam BW_ADDR = 32;
localparam BW_AXI_DATA = 128;
localparam BW_AXI_TID = `REQUIRED_BW_OF_SLAVE_TID;
localparam CAPA_PER_PORT = 16384;  // in bytes
localparam CELL_WIDTH = BW_AXI_DATA; // MUST greater than or equal to BW_AXI_DATA
localparam CELL_ARRAY_WIDTH = 512;
// localparam CELL_SIZE = 

`include "ervp_log_util.vf"
`include "ervp_bitwidth_util.vf"

localparam BW_BYTE_WEN = `NUM_BYTE(CELL_WIDTH);
localparam BW_CELL_ARRAY_BYTE_WEN = `NUM_BYTE(CELL_ARRAY_WIDTH);
localparam CELL_ARRAY_TOTAL_DEPTH = `DIVIDERU(CAPA_PER_PORT,BW_BYTE_WEN);
localparam BW_CELL_ARRAY_ROW_INDEX = REQUIRED_BITWIDTH_INDEX(CELL_ARRAY_TOTAL_DEPTH);

localparam NUM_AXI2RAM_PORT = `DIVIDERU(CAPACITY,CAPA_PER_PORT);
localparam NUM_CELL_PER_PORT = `DIVIDERU(CELL_ARRAY_WIDTH,CELL_WIDTH);

localparam CELL_SIZE = `DIVIDERU(CAPA_PER_PORT,NUM_CELL_PER_PORT);  // in bytes
localparam CELL_DEPTH = `DIVIDERU(CELL_SIZE,BW_BYTE_WEN);
localparam BW_CELL_INDEX = REQUIRED_BITWIDTH_INDEX(CELL_DEPTH);

localparam BW_BANK = REQUIRED_BITWIDTH_INDEX(NUM_CELL_PER_PORT);
localparam BW_ROW  = BW_CELL_ARRAY_ROW_INDEX - BW_BANK;


input wire clk, rstnn;

//AW Channel
input wire [BW_AXI_TID-1:0] rxawid;
input wire [BW_ADDR-1:0] rxawaddr;
input wire [`BW_AXI_ALEN-1:0] rxawlen;
input wire [`BW_AXI_ASIZE-1:0] rxawsize;
input wire [`BW_AXI_ABURST-1:0] rxawburst;
input wire rxawvalid;
output wire rxawready;

//W Channel
input wire [BW_AXI_TID-1:0] rxwid;
input wire [BW_AXI_DATA-1:0] rxwdata;
input wire [`BW_AXI_WSTRB(BW_AXI_DATA)-1:0] rxwstrb;
input wire rxwlast;
input wire rxwvalid;
output wire rxwready;

//B Channel
output wire [BW_AXI_TID-1:0] rxbid;
output wire [`BW_AXI_BRESP-1:0] rxbresp;
output wire rxbvalid;
input wire rxbready;

//AR Channel
input wire [BW_AXI_TID-1:0] rxarid;
input wire [BW_ADDR-1:0] rxaraddr;
input wire [`BW_AXI_ALEN-1:0] rxarlen;
input wire [`BW_AXI_ASIZE-1:0] rxarsize;
input wire [`BW_AXI_ABURST-1:0] rxarburst;
input wire rxarvalid;
output wire rxarready;

//R Channel
output wire [BW_AXI_TID-1:0] rxrid;
output wire [BW_AXI_DATA-1:0] rxrdata;
output wire [`BW_AXI_RRESP-1:0] rxrresp;
output wire rxrlast;
output wire rxrvalid;
input wire rxrready;

input wire [NUM_AXI2RAM_PORT-1:0] int_renable_list;
output wire [CELL_ARRAY_WIDTH*NUM_AXI2RAM_PORT-1:0] int_rdata_list;
output wire [NUM_AXI2RAM_PORT-1:0] int_rvalid_list;
input wire [BW_CELL_INDEX*NUM_AXI2RAM_PORT-1:0] int_index_list;
input wire [NUM_AXI2RAM_PORT-1:0] int_wenable_list;
input wire [BW_CELL_ARRAY_BYTE_WEN*NUM_AXI2RAM_PORT-1:0] int_byte_enable_list;
input wire [CELL_ARRAY_WIDTH*NUM_AXI2RAM_PORT-1:0] int_wdata_list; 


/////////////
/* signals */
/////////////

genvar i;

wire [NUM_AXI2RAM_PORT-1:0] axi2ram_select_list;
wire [BW_CELL_ARRAY_ROW_INDEX*NUM_AXI2RAM_PORT-1:0] axi2ram_index_list;
wire [NUM_AXI2RAM_PORT-1:0] axi2ram_enable_list;
wire [NUM_AXI2RAM_PORT-1:0] axi2ram_wenable_list;
wire [BW_BYTE_WEN*NUM_AXI2RAM_PORT-1:0] axi2ram_wenable_byte_list;
wire [BW_AXI_DATA*NUM_AXI2RAM_PORT-1:0] axi2ram_wenable_bit_list;
wire [BW_AXI_DATA*NUM_AXI2RAM_PORT-1:0] axi2ram_wdata_list;
wire [NUM_AXI2RAM_PORT-1:0] axi2ram_renable_list;
wire [BW_AXI_DATA*NUM_AXI2RAM_PORT-1:0] axi2ram_rdata_list;
wire [NUM_AXI2RAM_PORT-1:0] axi2ram_stall_list = 0;

wire [BW_CELL_ARRAY_ROW_INDEX-1:0] axi2ram_index [NUM_AXI2RAM_PORT-1:0];
wire axi2ram_enable [NUM_AXI2RAM_PORT-1:0];
wire axi2ram_wenable [NUM_AXI2RAM_PORT-1:0];
wire [BW_BYTE_WEN-1:0] axi2ram_wenable_byte [NUM_AXI2RAM_PORT-1:0];
wire [BW_AXI_DATA-1:0] axi2ram_wenable_bit [NUM_AXI2RAM_PORT-1:0];
wire [BW_AXI_DATA-1:0] axi2ram_wdata [NUM_AXI2RAM_PORT-1:0];
wire axi2ram_renable [NUM_AXI2RAM_PORT-1:0];
wire [BW_AXI_DATA-1:0] axi2ram_rdata [NUM_AXI2RAM_PORT-1:0];

wire [BW_BANK-1:0]             axi_bank_sel   [NUM_AXI2RAM_PORT-1:0];
wire [BW_ROW-1:0]              axi_row_index  [NUM_AXI2RAM_PORT-1:0];

////////////
/* logics */
////////////

ERVP_SPSRAM_CONTROLLER_AXI
#(
	.BW_ADDR(BW_ADDR),
	.BW_AXI_DATA(BW_AXI_DATA),
	.BW_AXI_TID(BW_AXI_TID),
	.BASEADDR(0),
	.CELL_SIZE(CAPA_PER_PORT),
  .CELL_WIDTH(CELL_WIDTH),
	.NUM_CELL(NUM_AXI2RAM_PORT)
)
i_controller
(
	.clk(clk),
	.rstnn(rstnn),
  .enable(1'b 1),

	.rxawid(rxawid),
	.rxawaddr(rxawaddr),
	.rxawlen(rxawlen),
	.rxawsize(rxawsize),
	.rxawburst(rxawburst),
	.rxawvalid(rxawvalid),
	.rxawready(rxawready),

	.rxwid(rxwid),
	.rxwdata(rxwdata),
	.rxwstrb(rxwstrb),
	.rxwlast(rxwlast),
	.rxwvalid(rxwvalid),
	.rxwready(rxwready),

	.rxbid(rxbid),
	.rxbresp(rxbresp),
	.rxbvalid(rxbvalid),
	.rxbready(rxbready),

	.rxarid(rxarid),
	.rxaraddr(rxaraddr),
	.rxarlen(rxarlen),
	.rxarsize(rxarsize),
	.rxarburst(rxarburst),
	.rxarvalid(rxarvalid),
	.rxarready(rxarready),

	.rxrid(rxrid),
	.rxrdata(rxrdata),
	.rxrresp(rxrresp),
	.rxrlast(rxrlast),
	.rxrvalid(rxrvalid),
	.rxrready(rxrready),

	.sscell_select_list(axi2ram_select_list),
	.sscell_index_list(axi2ram_index_list),
	.sscell_enable_list(axi2ram_enable_list),
	.sscell_wenable_list(axi2ram_wenable_list),
	.sscell_wenable_byte_list(axi2ram_wenable_byte_list),
	.sscell_wenable_bit_list(axi2ram_wenable_bit_list),
	.sscell_wdata_list(axi2ram_wdata_list),
	.sscell_renable_list(axi2ram_renable_list),
	.sscell_rdata_list(axi2ram_rdata_list),
	.sscell_stall_list(axi2ram_stall_list)
);

generate
for(i=0; i<NUM_AXI2RAM_PORT; i=i+1)
begin : generate_port_signals
	assign axi2ram_index[i] = axi2ram_index_list[BW_CELL_ARRAY_ROW_INDEX*(i+1)-1 -:BW_CELL_ARRAY_ROW_INDEX];
	assign axi2ram_enable[i] = axi2ram_enable_list[i];
	assign axi2ram_wenable[i] = axi2ram_wenable_list[i];
	assign axi2ram_wenable_byte[i] = axi2ram_wenable_byte_list[BW_BYTE_WEN*(i+1)-1 -:BW_BYTE_WEN];
	assign axi2ram_wenable_bit[i] = axi2ram_wenable_bit_list[BW_AXI_DATA*(i+1)-1 -:BW_AXI_DATA];
	assign axi2ram_wdata[i] = axi2ram_wdata_list[BW_AXI_DATA*(i+1)-1 -:BW_AXI_DATA];
	assign axi2ram_renable[i] = axi2ram_renable_list[i];
	assign axi2ram_rdata_list[BW_AXI_DATA*(i+1)-1 -:BW_AXI_DATA] = axi2ram_rdata[i];
end
endgenerate

generate
for(i=0; i<NUM_AXI2RAM_PORT; i=i+1) begin : g_addr_decode
	assign axi_bank_sel[i]  = axi2ram_index[i][BW_CELL_ARRAY_ROW_INDEX-1 -: BW_BANK];
	assign axi_row_index[i] = axi2ram_index[i][BW_ROW-1:0];
end
endgenerate
genvar j;
integer k;

reg [NUM_AXI2RAM_PORT-1:0] axi_rvalid;
reg [NUM_AXI2RAM_PORT-1:0] int_rvalid;
// Read valid delay
always @(posedge clk or negedge rstnn) begin
	if(!rstnn) begin
		axi_rvalid <= 0;
		int_rvalid <= 0;
	end else begin
		for(k=0; k<NUM_AXI2RAM_PORT; k=k+1) begin
			axi_rvalid[k] <= axi2ram_renable[k];
			int_rvalid[k] <= int_renable_list[k];
		end
	end
end

generate
for(i=0; i<NUM_AXI2RAM_PORT; i=i+1) begin : g_port

	wire                    			int_ren   = int_renable_list[i];
	wire                   				int_wen   = int_wenable_list[i];
	wire [BW_CELL_INDEX-1:0]          	int_idx   = int_index_list[BW_CELL_INDEX*(i+1)-1 -: BW_CELL_INDEX];
	wire [CELL_ARRAY_WIDTH-1:0]       	int_wdata = int_wdata_list[CELL_ARRAY_WIDTH*(i+1)-1 -: CELL_ARRAY_WIDTH];
	wire [BW_CELL_ARRAY_BYTE_WEN-1:0] 	int_be    = int_byte_enable_list[BW_CELL_ARRAY_BYTE_WEN*(i+1)-1 -: BW_CELL_ARRAY_BYTE_WEN];

	wire axi_busy = axi2ram_enable[i];

	wire grant_internal = (int_ren | int_wen) & ~axi_busy;

	wire [BW_ROW-1:0] side_row = int_idx[BW_ROW-1:0];

	wire [BW_AXI_DATA-1:0] bank_rdata_axi  [0:NUM_CELL_PER_PORT-1];
	wire [BW_AXI_DATA-1:0] bank_rdata_internal [0:NUM_CELL_PER_PORT-1];

	// Read data mux for AXI side
	reg [BW_BANK-1:0] axi_bank_sel_q;
	always @(posedge clk) begin
		if(axi2ram_renable[i])
			axi_bank_sel_q <= axi_bank_sel[i];
	end

	for(j=0; j<NUM_CELL_PER_PORT; j=j+1) begin : g_bank
		
		// Bank select signals for AXI
		wire axi_bank_sel_1b = (axi_bank_sel[i] == j);

		// Arbitration: AXI has priority
		wire bank_ren    = (axi2ram_renable[i] & axi_bank_sel_1b) | (int_ren & grant_internal);
		wire bank_wen   = (axi2ram_wenable[i] & axi_bank_sel_1b) | (int_wen & grant_internal);
		wire [BW_ROW-1:0] bank_addr = (axi2ram_enable[i] & axi_bank_sel_1b) ? axi_row_index[i] : side_row;
		
		// Write data and byte enable mux
		wire [BW_AXI_DATA-1:0] bank_wdata;
		wire [BW_BYTE_WEN-1:0] bank_be;
		
		// For AXI: use AXI data, For Internal: use corresponding slice from 512-bit data
		assign bank_wdata = (axi2ram_enable[i]) ? 
							axi2ram_wdata[i] : 
							int_wdata[BW_AXI_DATA*(j+1)-1 -: BW_AXI_DATA];
		
		assign bank_be = (axi2ram_enable[i] & axi_bank_sel_1b) ? 
						axi2ram_wenable_byte[i] : 
						int_be[BW_BYTE_WEN*(j+1)-1 -: BW_BYTE_WEN];

		// SRAM instance using ERVP memory cell
		wire [BW_AXI_DATA-1:0] bank_rdata;
		// wire bank_ren = bank_en & ~bank_wen;
		
		ERVP_MEMORY_CELL_1R1W #(
			.DEPTH(CELL_DEPTH),
			.WIDTH(BW_AXI_DATA),
			.BW_INDEX(BW_ROW),
			.USE_SINGLE_INDEX(1),
			.USE_SUBWORD_ENABLE(1),
			.BW_SUBWORD(8)
		) i_cell (
			.clk(clk),
			.rstnn(rstnn),
			.index(bank_addr),
			.windex(bank_addr),
			.wenable(bank_wen),
			.wpermit(bank_be),
			.wdata(bank_wdata),
			.rindex(bank_addr),
			.rdata_asynch(),
			.renable(bank_ren),
			.rdata_synch(bank_rdata)
		);

		assign bank_rdata_axi[j] = bank_rdata;
		assign bank_rdata_internal[j] = bank_rdata;
	end

	// Output read data assembly for AXI
	assign axi2ram_rdata[i] = bank_rdata_axi[axi_bank_sel_q];

	// Output read data assembly for internal side - concatenate all banks
	wire [CELL_ARRAY_WIDTH-1:0] int_rdata_full;
	for(j=0; j<NUM_CELL_PER_PORT; j=j+1) begin : g_rdata_assembly
		assign int_rdata_full[BW_AXI_DATA*(j+1)-1 -: BW_AXI_DATA] = bank_rdata_internal[j];
	end
	
	assign int_rdata_list[CELL_ARRAY_WIDTH*(i+1)-1 -: CELL_ARRAY_WIDTH] = int_rdata_full;
	assign int_rvalid_list[i] = int_rvalid[i];
end
endgenerate

endmodule