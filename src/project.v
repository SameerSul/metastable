/*
 * Metastable: programmable protocol emulator
 * Copyright (c) 2026 Sameer Suleman
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two PIO-style state machines + shared 32x16 instruction memory,
 * SPI slave host interface, 16-entry GPIO space:
 *   GPIO 0-7   bidirectional (uio)
 *   GPIO 8-12  input-only   (ui[3:7])
 *   GPIO 13-15 output-only  (uo[2:4])
 *
 * SPI register map (7-bit address, auto-increment except TXF/RXF):
 *   0x00-0x3F IMEM        byte access: 2i = instr[i][7:0], 2i+1 = [15:8]
 *   0x40 CTRL             [0] SM0_EN [1] SM1_EN [4] SM0_RESTART [5] SM1_RESTART
 *   0x41 FSTAT            [0] TX0_E [1] TX0_F [2] RX0_E [3] RX0_F [7:4] SM1
 *   0x42 IRQ              read flags / write 1 to clear
 *   0x43 IRQ_MASK         HOST_IRQ pin = |(IRQ & MASK)
 *   0x44 GPIO_IN_L        pins 7:0
 *   0x45 GPIO_IN_H        pins 15:8
 *   0x46 PC0  0x47 PC1    current program counters (debug)
 *   0x48 FLEVEL0  0x49 FLEVEL1   [3:0] TX fill level [7:4] RX fill level
 *   0x50-0x5E SM0, 0x60-0x6E SM1:
 *     +0 CLKDIV_INT_L  +1 CLKDIV_INT_H  +2 CLKDIV_FRAC
 *     +3 WRAP_TOP      +4 WRAP_BOTTOM
 *     +5 SHIFTCTRL     [0] autopull [1] autopush [2] out_right [3] in_right
 *     +6 THRESH        [3:0] pull (0=8)  [7:4] push (0=8)
 *     +7 PIN_OUT       [3:0] base [7:4] count
 *     +8 PIN_SET       [3:0] base [6:4] count
 *     +9 PIN_IN        [3:0] base
 *     +A PIN_SIDE      [3:0] base [5:4] count [6] opt [7] pindir
 *     +B JMP_PIN       [3:0] pin
 *     +C TXF (write)   +D RXF (read)
 *     +E STATUS_CFG    MOV STATUS = all-ones while sel FIFO level < N:
 *                      [3:0] N  [4] sel (0 TX, 1 RX); reset 0x01 = TX empty
 */

`default_nettype none

module tt_um_sulems6_metastable (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------------------------
  // GPIO input synchronizers
  // ------------------------------------------------------------------
  reg [12:0] gin_q1, gin_q2;
  always @(posedge clk) begin
    if (!rst_n) begin
      gin_q1 <= 13'd0;
      gin_q2 <= 13'd0;
    end else begin
      gin_q1 <= {ui_in[7:3], uio_in};
      gin_q2 <= gin_q1;
    end
  end

  reg  [15:0] gpio_out_q, gpio_oe_q;
  wire [15:0] gpio_in_full = {gpio_out_q[15:13], gin_q2};

  // ------------------------------------------------------------------
  // SPI slave
  // ------------------------------------------------------------------
  wire [6:0] spi_addr;
  wire [7:0] spi_wdata;
  wire       spi_wr, spi_rd;
  reg  [7:0] spi_rdata;
  wire       spi_miso;

  wire spi_addr_is_sm  = (spi_addr[6:4] == 3'b101) || (spi_addr[6:4] == 3'b110);
  wire spi_hold        = spi_addr_is_sm &&
                         (spi_addr[3:0] == 4'hC || spi_addr[3:0] == 4'hD);

  spi_slave u_spi (
      .clk(clk), .rst_n(rst_n),
      .sck(ui_in[0]), .cs_n(ui_in[1]), .mosi(ui_in[2]), .miso(spi_miso),
      .addr(spi_addr), .wdata(spi_wdata),
      .wr_en(spi_wr), .rd_en(spi_rd),
      .rdata(spi_rdata), .hold_addr(spi_hold)
  );

  // ------------------------------------------------------------------
  // instruction memory: 32 x 16
  // ------------------------------------------------------------------
  reg [7:0] imem_l [0:31];
  reg [7:0] imem_h [0:31];

  always @(posedge clk) begin
    if (spi_wr && !spi_addr[6]) begin
      if (spi_addr[0]) imem_h[spi_addr[5:1]] <= spi_wdata;
      else             imem_l[spi_addr[5:1]] <= spi_wdata;
    end
  end

  // ------------------------------------------------------------------
  // control / status registers
  // ------------------------------------------------------------------
  reg  [1:0] sm_en;
  reg  [3:0] irq_mask;
  reg  [3:0] irq_flags;

  wire       ctrl_wr  = spi_wr && (spi_addr == 7'h40);
  wire [1:0] restart  = {2{ctrl_wr}} & spi_wdata[5:4];

  // per-SM configuration registers
  reg [7:0]  div_int_l  [0:1];
  reg [7:0]  div_int_h  [0:1];
  reg [7:0]  div_frac   [0:1];
  reg [4:0]  wrap_top   [0:1];
  reg [4:0]  wrap_bot   [0:1];
  reg [3:0]  shiftctrl  [0:1]; // {in_right, out_right, autopush, autopull}
  reg [7:0]  thresh     [0:1];
  reg [7:0]  pin_out    [0:1];
  reg [6:0]  pin_set    [0:1];
  reg [3:0]  pin_in     [0:1];
  reg [7:0]  pin_side   [0:1];
  reg [3:0]  jmp_pin    [0:1];
  reg [4:0]  status_cfg [0:1]; // [3:0] level N, [4] sel (0 TX, 1 RX)

  wire       cfg_sm1 = (spi_addr[6:4] == 3'b110);
  integer k;

  always @(posedge clk) begin
    if (!rst_n) begin
      sm_en    <= 2'b00;
      irq_mask <= 4'd0;
      for (k = 0; k < 2; k = k + 1) begin
        div_int_l[k] <= 8'd1;
        div_int_h[k] <= 8'd0;
        div_frac[k]  <= 8'd0;
        wrap_top[k]  <= 5'd31;
        wrap_bot[k]  <= 5'd0;
        shiftctrl[k] <= 4'b1100; // shift right both directions
        thresh[k]    <= 8'd0;    // 8/8
        pin_out[k]   <= 8'd0;
        pin_set[k]   <= 7'd0;
        pin_in[k]    <= 4'd0;
        pin_side[k]  <= 8'd0;
        jmp_pin[k]   <= 4'd0;
        status_cfg[k] <= 5'h01; // TX level < 1 (TX empty)
      end
    end else if (spi_wr) begin
      if (spi_addr == 7'h40) sm_en    <= spi_wdata[1:0];
      if (spi_addr == 7'h43) irq_mask <= spi_wdata[3:0];
      if (spi_addr_is_sm) begin
        case (spi_addr[3:0])
          4'h0: div_int_l[cfg_sm1] <= spi_wdata;
          4'h1: div_int_h[cfg_sm1] <= spi_wdata;
          4'h2: div_frac[cfg_sm1]      <= spi_wdata;
          4'h3: wrap_top[cfg_sm1]      <= spi_wdata[4:0];
          4'h4: wrap_bot[cfg_sm1]      <= spi_wdata[4:0];
          4'h5: shiftctrl[cfg_sm1]     <= spi_wdata[3:0];
          4'h6: thresh[cfg_sm1]        <= spi_wdata;
          4'h7: pin_out[cfg_sm1]       <= spi_wdata;
          4'h8: pin_set[cfg_sm1]       <= spi_wdata[6:0];
          4'h9: pin_in[cfg_sm1]        <= spi_wdata[3:0];
          4'hA: pin_side[cfg_sm1]      <= spi_wdata;
          4'hB: jmp_pin[cfg_sm1]       <= spi_wdata[3:0];
          4'hE: status_cfg[cfg_sm1]    <= spi_wdata[4:0];
          default: ;
        endcase
      end
    end
  end

  // ------------------------------------------------------------------
  // FIFOs
  // ------------------------------------------------------------------
  wire [7:0] tx_rdata [0:1];
  wire       tx_empty [0:1];
  wire       tx_full  [0:1];
  wire       tx_pop   [0:1];
  wire [7:0] rx_rdata [0:1];
  wire       rx_empty [0:1];
  wire       rx_full  [0:1];
  wire [7:0] rx_wdata [0:1];
  wire       rx_push  [0:1];
  wire [3:0] tx_level [0:1];
  wire [3:0] rx_level [0:1];

  wire [1:0] txf_wr = {spi_wr && (spi_addr == 7'h6C),
                       spi_wr && (spi_addr == 7'h5C)};
  wire [1:0] rxf_rd = {spi_rd && (spi_addr == 7'h6D),
                       spi_rd && (spi_addr == 7'h5D)};

  genvar g;
  generate
    for (g = 0; g < 2; g = g + 1) begin : fifos
      pio_fifo u_tx (
          .clk(clk), .rst_n(rst_n), .flush(restart[g]),
          .push(txf_wr[g]), .wdata(spi_wdata),
          .pop(tx_pop[g]), .rdata(tx_rdata[g]),
          .full(tx_full[g]), .empty(tx_empty[g]), .level(tx_level[g])
      );
      pio_fifo u_rx (
          .clk(clk), .rst_n(rst_n), .flush(restart[g]),
          .push(rx_push[g]), .wdata(rx_wdata[g]),
          .pop(rxf_rd[g]), .rdata(rx_rdata[g]),
          .full(rx_full[g]), .empty(rx_empty[g]), .level(rx_level[g])
      );
    end
  endgenerate

  // ------------------------------------------------------------------
  // clock dividers + state machines
  // ------------------------------------------------------------------
  wire [1:0]  tick;
  wire [4:0]  pc      [0:1];
  wire [15:0] instr   [0:1];
  wire [15:0] pmask   [0:1];
  wire [15:0] pdata   [0:1];
  wire [15:0] dmask   [0:1];
  wire [15:0] ddata   [0:1];
  wire [3:0]  iset    [0:1];
  wire [3:0]  iclr    [0:1];
  wire [1:0]  sm_stall;

  assign instr[0] = {imem_h[pc[0]], imem_l[pc[0]]};
  assign instr[1] = {imem_h[pc[1]], imem_l[pc[1]]};

  // MOV STATUS: all-ones while the selected FIFO's level is below N
  wire [1:0] status_sel;
  generate
    for (g = 0; g < 2; g = g + 1) begin : status
      assign status_sel[g] = (status_cfg[g][4] ? rx_level[g] : tx_level[g])
                             < status_cfg[g][3:0];
    end
  endgenerate

  generate
    for (g = 0; g < 2; g = g + 1) begin : sms
      pio_clkdiv u_div (
          .clk(clk), .rst_n(rst_n), .en(sm_en[g]), .restart(restart[g]),
          .div_int({div_int_h[g], div_int_l[g]}), .div_frac(div_frac[g]), .tick(tick[g])
      );
      pio_sm u_sm (
          .clk(clk), .rst_n(rst_n), .restart(restart[g]), .tick(tick[g]),
          .pc_out(pc[g]), .instr(instr[g]),
          .wrap_top(wrap_top[g]), .wrap_bottom(wrap_bot[g]),
          .autopull(shiftctrl[g][0]), .autopush(shiftctrl[g][1]),
          .out_shift_right(shiftctrl[g][2]), .in_shift_right(shiftctrl[g][3]),
          .pull_thresh(thresh[g][3:0]), .push_thresh(thresh[g][7:4]),
          .out_base(pin_out[g][3:0]), .out_count(pin_out[g][7:4]),
          .set_base(pin_set[g][3:0]), .set_count(pin_set[g][6:4]),
          .in_base(pin_in[g]),
          .side_base(pin_side[g][3:0]), .side_count(pin_side[g][5:4]),
          .side_opt(pin_side[g][6]), .side_pindir(pin_side[g][7]),
          .jmp_pin(jmp_pin[g]),
          .status_sel(status_sel[g]),
          .gpio_in(gpio_in_full),
          .pin_wr_mask(pmask[g]), .pin_wr_data(pdata[g]),
          .dir_wr_mask(dmask[g]), .dir_wr_data(ddata[g]),
          .tx_rdata(tx_rdata[g]), .tx_empty(tx_empty[g]), .tx_pop(tx_pop[g]),
          .rx_wdata(rx_wdata[g]), .rx_push(rx_push[g]), .rx_full(rx_full[g]),
          .irq_flags(irq_flags), .irq_set(iset[g]), .irq_clr(iclr[g]),
          .stalled(sm_stall[g])
      );
    end
  endgenerate

  // ------------------------------------------------------------------
  // IRQ flags (SM set/clear + host write-1-to-clear)
  // ------------------------------------------------------------------
  wire [3:0] host_w1c = (spi_wr && spi_addr == 7'h42) ? spi_wdata[3:0] : 4'd0;

  always @(posedge clk) begin
    if (!rst_n) irq_flags <= 4'd0;
    else irq_flags <= (irq_flags | iset[0] | iset[1])
                      & ~(iclr[0] | iclr[1] | host_w1c);
  end

  // ------------------------------------------------------------------
  // pin router: SM1 wins conflicts, pins hold last driven value
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n) begin
      gpio_out_q <= 16'd0;
      gpio_oe_q  <= 16'd0;
    end else begin
      gpio_out_q <= (gpio_out_q & ~(pmask[0] | pmask[1]))
                  | (pdata[0] & pmask[0] & ~pmask[1])
                  | (pdata[1] & pmask[1]);
      gpio_oe_q  <= (gpio_oe_q & ~(dmask[0] | dmask[1]))
                  | (ddata[0] & dmask[0] & ~dmask[1])
                  | (ddata[1] & dmask[1]);
    end
  end

  // ------------------------------------------------------------------
  // SPI read mux
  // ------------------------------------------------------------------
  wire rd_sm1 = (spi_addr[6:4] == 3'b110);

  always @* begin
    if (!spi_addr[6]) begin
      spi_rdata = spi_addr[0] ? imem_h[spi_addr[5:1]] : imem_l[spi_addr[5:1]];
    end else if (spi_addr[5:4] == 2'b00) begin
      case (spi_addr[3:0])
        4'h0: spi_rdata = {2'd0, restart, 2'd0, sm_en};
        4'h1: spi_rdata = {rx_full[1], rx_empty[1], tx_full[1], tx_empty[1],
                           rx_full[0], rx_empty[0], tx_full[0], tx_empty[0]};
        4'h2: spi_rdata = {4'd0, irq_flags};
        4'h3: spi_rdata = {4'd0, irq_mask};
        4'h4: spi_rdata = gpio_in_full[7:0];
        4'h5: spi_rdata = gpio_in_full[15:8];
        4'h6: spi_rdata = {3'd0, pc[0]};
        4'h7: spi_rdata = {3'd0, pc[1]};
        4'h8: spi_rdata = {rx_level[0], tx_level[0]};
        4'h9: spi_rdata = {rx_level[1], tx_level[1]};
        default: spi_rdata = 8'd0;
      endcase
    end else if (spi_addr_is_sm) begin
      case (spi_addr[3:0])
        4'h0: spi_rdata = div_int_l[rd_sm1];
        4'h1: spi_rdata = div_int_h[rd_sm1];
        4'h2: spi_rdata = div_frac[rd_sm1];
        4'h3: spi_rdata = {3'd0, wrap_top[rd_sm1]};
        4'h4: spi_rdata = {3'd0, wrap_bot[rd_sm1]};
        4'h5: spi_rdata = {4'd0, shiftctrl[rd_sm1]};
        4'h6: spi_rdata = thresh[rd_sm1];
        4'h7: spi_rdata = pin_out[rd_sm1];
        4'h8: spi_rdata = {1'b0, pin_set[rd_sm1]};
        4'h9: spi_rdata = {4'd0, pin_in[rd_sm1]};
        4'hA: spi_rdata = pin_side[rd_sm1];
        4'hB: spi_rdata = {4'd0, jmp_pin[rd_sm1]};
        4'hD: spi_rdata = rx_rdata[rd_sm1];
        4'hE: spi_rdata = {3'd0, status_cfg[rd_sm1]};
        default: spi_rdata = 8'd0;
      endcase
    end else begin
      spi_rdata = 8'd0;
    end
  end

  // ------------------------------------------------------------------
  // outputs
  // ------------------------------------------------------------------
  reg [23:0] heartbeat;
  always @(posedge clk) begin
    if (!rst_n) heartbeat <= 24'd0;
    else heartbeat <= heartbeat + 24'd1;
  end

  assign uo_out[0] = spi_miso;
  assign uo_out[1] = |(irq_flags & irq_mask);
  assign uo_out[2] = gpio_out_q[13];
  assign uo_out[3] = gpio_out_q[14];
  assign uo_out[4] = gpio_out_q[15];
  assign uo_out[5] = sm_stall[0];
  assign uo_out[6] = sm_stall[1];
  assign uo_out[7] = heartbeat[23];

  assign uio_out = gpio_out_q[7:0];
  assign uio_oe  = gpio_oe_q[7:0];

  wire _unused = &{ena, 1'b0};

endmodule
