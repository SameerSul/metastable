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

`ifdef FORMAL
  // Formal timing theorem (yosys-smtbmc, see formal/run.sh): under a
  // stable configuration, every tick-to-tick gap is exactly
  // div_int + carry, where carry follows the accumulator's mod-256
  // walk — so any 256 consecutive ticks span exactly
  // 256*div_int + div_frac cycles: zero cumulative drift, +-1 cycle
  // jitter. This is the property the USB LS bit clock relies on.
  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @* if (!f_past_valid) assume (!rst_n);

  // the host only reprograms CLKDIV while the SM is disabled, and the
  // theorem is about the free-running divider
  reg [15:0] f_int_q;
  reg [7:0]  f_frac_q;
  always @(posedge clk) begin
    f_int_q  <= div_int;
    f_frac_q <= div_frac;
  end
  always @* if (f_past_valid) begin
    assume (div_int == f_int_q);
    assume (div_frac == f_frac_q);
    assume (en);
    assume (!restart);
  end

  wire [16:0] f_base  = (div_int == 16'd0) ? 17'd65536 : {1'b0, div_int};
  wire        f_carry = (div_frac != 8'd0) && (f_acc_prev < div_frac);
  reg  [17:0] f_gap = 18'd0;
  reg  [7:0]  f_acc_prev = 8'd0;
  reg         f_seen = 1'b0;

  always @(posedge clk) begin
    if (!rst_n) begin
      f_gap  <= 18'd0;
      f_seen <= 1'b0;
    end else if (tick) begin
      f_seen     <= 1'b1;
      f_acc_prev <= acc;
      f_gap      <= 18'd1;
    end else begin
      f_gap <= f_gap + 18'd1;
    end
  end

  always @* if (f_past_valid && rst_n) begin
    assert (cnt >= 17'd1);
    assert ({1'b0, cnt} <= {1'b0, f_base} + 18'd1);
    if (f_seen) begin
      if (tick)
        assert (f_gap == {1'b0, f_base} + {17'd0, f_carry});
      else begin
        assert (acc == f_acc_prev);
        assert (f_gap + {1'b0, cnt} == {1'b0, f_base} + {17'd0, f_carry});
      end
    end
  end
`endif

endmodule
