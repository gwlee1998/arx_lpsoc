module TwoPortMem_1(
  input          clock,
  input          io_wr_en,
  input  [15:0]  io_wr_addr,
  input  [127:0] io_wr_data,
  input          io_rd_en,
  input  [15:0]  io_rd_addr,
  output [127:0] io_rd_data
);
`ifdef RANDOMIZE_MEM_INIT
  reg [127:0] _RAND_0;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [127:0] mem [0:511]; // @[SyncQueue.scala 498:24]
  wire [127:0] mem_io_rd_data_MPORT_data; // @[SyncQueue.scala 498:24]
  wire [8:0] mem_io_rd_data_MPORT_addr; // @[SyncQueue.scala 498:24]
  wire [127:0] mem_MPORT_data; // @[SyncQueue.scala 498:24]
  wire [8:0] mem_MPORT_addr; // @[SyncQueue.scala 498:24]
  wire  mem_MPORT_mask; // @[SyncQueue.scala 498:24]
  wire  mem_MPORT_en; // @[SyncQueue.scala 498:24]
  reg  mem_io_rd_data_MPORT_en_pipe_0;
  reg [8:0] mem_io_rd_data_MPORT_addr_pipe_0;
  assign mem_io_rd_data_MPORT_addr = mem_io_rd_data_MPORT_addr_pipe_0;
  assign mem_io_rd_data_MPORT_data = mem[mem_io_rd_data_MPORT_addr]; // @[SyncQueue.scala 498:24]
  assign mem_MPORT_data = io_wr_data;
  assign mem_MPORT_addr = io_wr_addr[8:0];
  assign mem_MPORT_mask = 1'h1;
  assign mem_MPORT_en = io_wr_en;
  assign io_rd_data = mem_io_rd_data_MPORT_data; // @[SyncQueue.scala 504:20 SyncQueue.scala 505:16]
  always @(posedge clock) begin
    if(mem_MPORT_en & mem_MPORT_mask) begin
      mem[mem_MPORT_addr] <= mem_MPORT_data; // @[SyncQueue.scala 498:24]
    end
    mem_io_rd_data_MPORT_en_pipe_0 <= io_rd_en;
    if (io_rd_en) begin
      mem_io_rd_data_MPORT_addr_pipe_0 <= io_rd_addr[8:0];
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {4{`RANDOM}};
  for (initvar = 0; initvar < 512; initvar = initvar+1)
    mem[initvar] = _RAND_0[127:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  mem_io_rd_data_MPORT_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  mem_io_rd_data_MPORT_addr_pipe_0 = _RAND_2[8:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule