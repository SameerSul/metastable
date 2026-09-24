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

`ifdef FORMAL
  // Formal properties (yosys-smtbmc, see formal/run.sh): structural
  // invariants proved by induction, plus a data-integrity proof via an
  // arbitrary watched slot - whatever value was pushed into a slot is
  // exactly what sits there (and is read out) while the slot is live.
  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @* if (!f_past_valid) assume (!rst_n);

  (* anyconst *) wire [2:0] f_slot;
  wire [2:0] f_off = f_slot - rptr;
  wire       f_occ = (count == 4'd8) || ({1'b0, f_off} < count);

  reg [7:0] f_val;
  reg       f_live = 1'b0;
  always @(posedge clk) begin
    if (!rst_n || flush) f_live <= 1'b0;
    else if (do_push && wptr == f_slot) begin
      f_val  <= wdata;
      f_live <= 1'b1;
    end else if (do_pop && rptr == f_slot) f_live <= 1'b0;
  end

  always @* begin
    if (f_past_valid && rst_n) begin
      assert (count <= 4'd8);
      assert (full  == (count == 4'd8));
      assert (empty == (count == 4'd0));
      assert (wptr - rptr == count[2:0]);
      if (f_live && f_occ) begin
        assert (mem[f_slot] == f_val);
        if (rptr == f_slot) assert (rdata == f_val);
      end
    end
  end
`endif

endmodule
