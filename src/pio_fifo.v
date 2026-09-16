/*
 * Metastable protocol emulator - 8-deep x 8-bit synchronous FIFO
 * Copyright (c) 2026 Sameer Suleman
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pio_fifo (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       flush,
    input  wire       push,
    input  wire [7:0] wdata,
    input  wire       pop,
    output wire [7:0] rdata,
    output wire       full,
    output wire       empty,
    output wire [3:0] level
);

  reg [7:0] mem [0:7];
  reg [2:0] rptr, wptr;
  reg [3:0] count;

  wire do_push = push && !full;
  wire do_pop  = pop  && !empty;

  assign rdata = mem[rptr];
  assign full  = (count == 4'd8);
  assign empty = (count == 4'd0);
  assign level = count;

  always @(posedge clk) begin
    if (!rst_n || flush) begin
      rptr  <= 3'd0;
      wptr  <= 3'd0;
      count <= 4'd0;
    end else begin
      if (do_push) begin
        mem[wptr] <= wdata;
        wptr      <= wptr + 3'd1;
      end
      if (do_pop) rptr <= rptr + 3'd1;
      case ({do_push, do_pop})
        2'b10:   count <= count + 4'd1;
        2'b01:   count <= count - 4'd1;
        default: count <= count;
      endcase
    end
  end

endmodule
