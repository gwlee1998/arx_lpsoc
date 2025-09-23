module SyncQueue(
  input          clock,
  input          reset,
  output         io_enq_ready,
  input          io_enq_valid,
  input  [127:0] io_enq_bits,
  input          io_deq_ready,
  output         io_deq_valid,
  output [127:0] io_deq_bits,
  output [7:0]   io_count
);
  wire  queue_clock; // @[SyncQueue.scala 47:23]
  wire  queue_reset; // @[SyncQueue.scala 47:23]
  wire  queue_io_enq_ready; // @[SyncQueue.scala 47:23]
  wire  queue_io_enq_valid; // @[SyncQueue.scala 47:23]
  wire [127:0] queue_io_enq_bits; // @[SyncQueue.scala 47:23]
  wire  queue_io_deq_ready; // @[SyncQueue.scala 47:23]
  wire  queue_io_deq_valid; // @[SyncQueue.scala 47:23]
  wire [127:0] queue_io_deq_bits; // @[SyncQueue.scala 47:23]
  wire [7:0] queue_io_count; // @[SyncQueue.scala 47:23]
  SyncQueue2PortMem queue ( // @[SyncQueue.scala 47:23]
    .clock(queue_clock),
    .reset(queue_reset),
    .io_enq_ready(queue_io_enq_ready),
    .io_enq_valid(queue_io_enq_valid),
    .io_enq_bits(queue_io_enq_bits),
    .io_deq_ready(queue_io_deq_ready),
    .io_deq_valid(queue_io_deq_valid),
    .io_deq_bits(queue_io_deq_bits),
    .io_count(queue_io_count)
  );
  assign io_enq_ready = queue_io_enq_ready; // @[SyncQueue.scala 48:8]
  assign io_deq_valid = queue_io_deq_valid; // @[SyncQueue.scala 48:8]
  assign io_deq_bits = queue_io_deq_bits; // @[SyncQueue.scala 48:8]
  assign io_count = queue_io_count; // @[SyncQueue.scala 48:8]
  assign queue_clock = clock;
  assign queue_reset = reset;
  assign queue_io_enq_valid = io_enq_valid; // @[SyncQueue.scala 48:8]
  assign queue_io_enq_bits = io_enq_bits; // @[SyncQueue.scala 48:8]
  assign queue_io_deq_ready = io_deq_ready; // @[SyncQueue.scala 48:8]
endmodule