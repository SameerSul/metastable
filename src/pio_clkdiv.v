/*
 * Metastable protocol emulator - fractional clock divider (16.8 fixed point)
 * Divide ratio = div_int + div_frac/256; div_int == 0 means 65536.
 * Copyright (c) 2026 Sameer Suleman
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pio_clkdiv (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire        restart,
    input  wire [15:0] div_int,
    input  wire [7:0]  div_frac,
    output reg         tick
);

  reg  [16:0] cnt;
  reg  [7:0]  acc;

  wire [8:0]  acc_next = {1'b0, acc} + {1'b0, div_frac};
  wire [16:0] base     = (div_int == 16'd0) ? 17'd65536 : {1'b0, div_int};
  wire [16:0] period   = base + {16'd0, acc_next[8]};

  always @(posedge clk) begin
    if (!rst_n || restart) begin
      cnt  <= 17'd1;
      acc  <= 8'd0;
      tick <= 1'b0;
    end else if (en) begin
      if (cnt <= 17'd1) begin
        tick <= 1'b1;
        acc  <= acc_next[7:0];
        cnt  <= period;
      end else begin
        tick <= 1'b0;
        cnt  <= cnt - 17'd1;
      end
    end else begin
      tick <= 1'b0;
    end
  end

endmodule
