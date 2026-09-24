/*
 * Metastable protocol emulator - SPI slave (mode 0), synchronous sampling
 * Copyright (c) 2026 Sameer Suleman
 * SPDX-License-Identifier: Apache-2.0
 *
 * Transaction: CS_N low, first byte = {RW, ADDR[6:0]} (RW=1 write),
 * then N data bytes. The address auto-increments after each data byte
 * unless hold_addr is asserted (FIFO data registers).
 *
 * Read data is fetched speculatively on the falling SCK edge that
 * starts each data byte; read side effects (rd_en, e.g. RX FIFO pop)
 * and the address increment commit only once the byte has been fully
 * clocked out, so an aborted or parked SCK never loses FIFO data.
 * Requires clk >= ~8x SCK.
 */

`default_nettype none

module spi_slave (
    input  wire       clk,
    input  wire       rst_n,
    // pads
    input  wire       sck,
    input  wire       cs_n,
    input  wire       mosi,
    output reg        miso,
    // register bus (clk domain)
    output reg  [6:0] addr,
    output reg  [7:0] wdata,
    output reg        wr_en,     // 1-cycle strobe, addr/wdata valid
    output reg        rd_en,     // 1-cycle strobe, rdata consumed this cycle
    input  wire [7:0] rdata,
    input  wire       hold_addr  // suppress auto-increment for current addr
);

  // 2FF synchronizers + edge detect
  reg [2:0] sck_q;
  reg [1:0] cs_q, mosi_q;
  always @(posedge clk) begin
    if (!rst_n) begin
      sck_q  <= 3'b000;
      cs_q   <= 2'b11;
      mosi_q <= 2'b00;
    end else begin
      sck_q  <= {sck_q[1:0], sck};
      cs_q   <= {cs_q[0], cs_n};
      mosi_q <= {mosi_q[0], mosi};
    end
  end
  wire sck_rise = (sck_q[1] && !sck_q[2]);
  wire sck_fall = (!sck_q[1] && sck_q[2]);
  wire cs_act   = !cs_q[1];
  wire mosi_s   = mosi_q[1];

  reg [2:0] bitcnt;
  reg       first_byte;
  reg       rw;
  reg [6:0] shreg_in;
  reg [7:0] shreg_out;
  reg       wr_req, rd_req, inc_req;

  wire [7:0] rx_byte = {shreg_in, mosi_s};

  always @(posedge clk) begin
    if (!rst_n) begin
      bitcnt     <= 3'd0;
      first_byte <= 1'b1;
      rw         <= 1'b0;
      shreg_in   <= 7'd0;
      shreg_out  <= 8'd0;
      miso       <= 1'b0;
      addr       <= 7'd0;
      wdata      <= 8'd0;
      wr_en      <= 1'b0;
      rd_en      <= 1'b0;
      wr_req     <= 1'b0;
      rd_req     <= 1'b0;
      inc_req    <= 1'b0;
    end else begin
      wr_en <= 1'b0;
      rd_en <= 1'b0;

      // pending-request pipeline (one SPI byte spans many clk cycles)
      if (inc_req) begin
        inc_req <= 1'b0;
        if (!hold_addr) addr <= addr + 7'd1;
      end
      if (wr_req) begin
        wr_req  <= 1'b0;
        wr_en   <= 1'b1;
        inc_req <= 1'b1;
      end
      if (rd_req) begin
        rd_req  <= 1'b0;
        rd_en   <= 1'b1;
        inc_req <= 1'b1;
      end

      if (!cs_act) begin
        bitcnt     <= 3'd0;
        first_byte <= 1'b1;
        miso       <= 1'b0;
      end else begin
        if (sck_rise) begin
          shreg_in <= rx_byte[6:0];
          bitcnt   <= bitcnt + 3'd1;
          if (bitcnt == 3'd7) begin
            if (first_byte) begin
              first_byte <= 1'b0;
              rw         <= rx_byte[7];
              addr       <= rx_byte[6:0];
            end else if (rw) begin
              wdata  <= rx_byte;
              wr_req <= 1'b1;
            end else begin
              rd_req <= 1'b1; // commit read side effects for completed byte
            end
          end
        end
        if (sck_fall) begin
          if (bitcnt == 3'd0 && !first_byte && !rw) begin
            // start of a read data byte: speculative fetch, present MSB
            miso      <= rdata[7];
            shreg_out <= {rdata[6:0], 1'b0};
          end else begin
            miso      <= shreg_out[7];
            shreg_out <= {shreg_out[6:0], 1'b0};
          end
        end
      end
    end
  end

`ifdef FORMAL
  // Formal properties (yosys-smtbmc, see formal/run.sh), proved with NO
  // assumptions on sck/cs_n/mosi - arbitrary pad waveforms, including
  // glitches and mid-byte aborts. The register-bus strobes stay
  // disciplined: single-cycle, mutually exclusive, never re-fired for
  // the same byte. This is what makes FIFO pops/pushes exactly-once.
  reg f_past_valid = 1'b0;
  always @(posedge clk) f_past_valid <= 1'b1;
  always @* if (!f_past_valid) assume (!rst_n);

  reg f_wr_q = 1'b0, f_rd_q = 1'b0;
  always @(posedge clk) begin
    f_wr_q <= wr_en;
    f_rd_q <= rd_en;
  end

  always @* if (f_past_valid && rst_n) begin
    assert (!(wr_en && rd_en));
    assert (!(wr_req && rd_req));
    assert (!(wr_en && f_wr_q));  // strobes are single-cycle
    assert (!(rd_en && f_rd_q));
    // a request or strobe can only be in flight right after a byte
    // completed, i.e. while bitcnt has wrapped to 0 - which is what
    // rules out double commits from any sck pattern
    if (wr_req || rd_req || wr_en || rd_en)
      assert (bitcnt == 3'd0);
  end
`endif

endmodule
