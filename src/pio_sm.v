/*
 * Metastable protocol emulator - programmable I/O state machine
 * Copyright (c) 2026 Sameer Suleman
 * SPDX-License-Identifier: Apache-2.0
 *
 * RP2040-PIO-inspired ISA, 8-bit datapath, 16-bit instructions:
 *   [15:13] opcode  [12:8] delay/side-set  [7:0] operands
 *
 *   000 JMP   cond[7:5] addr[4:0]   cond: 0 always,1 !X,2 X--,3 !Y,4 Y--,
 *                                         5 X!=Y,6 PIN,7 !OSRE
 *   001 WAIT  pol[7] src[6:5] idx[4:0]   src: 0 GPIO(abs),1 PIN(rel),2 IRQ
 *   010 IN    src[7:5] n[4:0]      src: 0 PINS,1 X,2 Y,3 NULL,6 ISR,7 OSR
 *   011 OUT   dst[7:5] n[4:0]      dst: 0 PINS,1 X,2 Y,3 NULL,4 PINDIRS,
 *                                       5 PC,6 ISR
 *   100 PUSH/PULL  pull[7] iffull_ifempty[6] block[5]
 *   101 MOV   dst[7:5] op[4:3] src[2:0]
 *                dst: 0 PINS,1 X,2 Y,4 EXEC,5 PC,6 ISR,7 OSR
 *                op:  0 copy,1 invert,2 bit-reverse
 *                src: 0 PINS,1 X,2 Y,3 NULL,5 STATUS,6 ISR,7 OSR
 *   110 IRQ   clr[6] wait[5] idx[1:0]
 *   111 SET   dst[7:5] imm[4:0]    dst: 0 PINS,1 X,2 Y,4 PINDIRS
 *
 * Shift counts n: 1..8 (0 or >8 means 8).
 * Side-set: side_count MSBs of the delay/side field drive side_base pins on
 * every attempt (even stalled). With side_opt, dss[4] is a per-instruction
 * enable and the side bits shift down one. With side_pindir, side-set
 * drives pin directions instead of pin values (open-drain protocols).
 * MOV EXEC injects {8'h00, src} as the next instruction (i.e. a JMP with
 * zero delay), enabling computed jumps from any 8-bit source.
 */

`default_nettype none

module pio_sm (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        restart,
    input  wire        tick,
    // instruction memory
    output wire [4:0]  pc_out,
    input  wire [15:0] instr,
    // configuration
    input  wire [4:0]  wrap_top,
    input  wire [4:0]  wrap_bottom,
    input  wire        autopull,
    input  wire        autopush,
    input  wire        out_shift_right,
    input  wire        in_shift_right,
    input  wire [3:0]  pull_thresh,   // 0 means 8
    input  wire [3:0]  push_thresh,   // 0 means 8
    input  wire [3:0]  out_base,
    input  wire [3:0]  out_count,     // 0..8
    input  wire [3:0]  set_base,
    input  wire [2:0]  set_count,     // 0..7
    input  wire [3:0]  in_base,
    input  wire [3:0]  side_base,
    input  wire [1:0]  side_count,    // 0..3
    input  wire        side_opt,      // dss[4] = per-instruction side enable
    input  wire        side_pindir,   // side-set drives pindirs, not pins
    input  wire [3:0]  jmp_pin,
    // pins
    input  wire [15:0] gpio_in,
    output reg  [15:0] pin_wr_mask,
    output reg  [15:0] pin_wr_data,
    output reg  [15:0] dir_wr_mask,
    output reg  [15:0] dir_wr_data,
    // MOV STATUS source (project-level FIFO level compare)
    input  wire        status_sel,
    // TX FIFO (host -> SM)
    input  wire [7:0]  tx_rdata,
    input  wire        tx_empty,
    output reg         tx_pop,
    // RX FIFO (SM -> host)
    output reg  [7:0]  rx_wdata,
    output reg         rx_push,
    input  wire        rx_full,
    // shared IRQ flags
    input  wire [3:0]  irq_flags,
    output reg  [3:0]  irq_set,
    output reg  [3:0]  irq_clr,
    // debug
    output reg         stalled
);

  // ------------------------------------------------------------------
  // architectural state
  // ------------------------------------------------------------------
  reg [4:0] pc;
  reg [7:0] x, y, osr, isr;
  reg [3:0] osr_cnt;   // bits already shifted out of OSR (8 = empty)
  reg [3:0] isr_cnt;   // bits shifted into ISR
  reg [4:0] delay_q;
  reg       exec_valid;
  reg [7:0] exec_byte;
  reg       irq_sent;

  assign pc_out = pc;

  // ------------------------------------------------------------------
  // helper functions
  // ------------------------------------------------------------------
  function [7:0] rev8(input [7:0] v);
    integer i;
    begin
      for (i = 0; i < 8; i = i + 1) rev8[i] = v[7-i];
    end
  endfunction

  function [15:0] pin_mask(input [3:0] base, input [3:0] cnt);
    integer i;
    begin
      pin_mask = 16'd0;
      for (i = 0; i < 8; i = i + 1)
        if (i < {28'd0, cnt}) pin_mask[({28'd0, base} + i) % 16] = 1'b1;
    end
  endfunction

  function [15:0] pin_spread(input [3:0] base, input [3:0] cnt, input [7:0] val);
    integer i;
    begin
      pin_spread = 16'd0;
      for (i = 0; i < 8; i = i + 1)
        if (i < {28'd0, cnt}) pin_spread[({28'd0, base} + i) % 16] = val[i];
    end
  endfunction

  function [7:0] bitmask8(input [3:0] n);
    begin
      bitmask8 = 8'hFF >> (4'd8 - n);
    end
  endfunction

  // ------------------------------------------------------------------
  // instruction selection and field decode
  // ------------------------------------------------------------------
  wire [15:0] ci  = exec_valid ? {8'h00, exec_byte} : instr;
  wire [2:0]  opc = ci[15:13];
  wire [4:0]  dss = ci[12:8];

  reg [4:0] delay_field;
  reg [2:0] side_val;
  reg       side_en;
  always @* begin
    if (side_opt && side_count != 2'd0) begin
      // dss[4] is the per-instruction side-set enable; side bits shift down
      side_en = dss[4];
      case (side_count)
        2'd1:    begin side_val = {2'b00, dss[3]};  delay_field = {2'b00, dss[2:0]}; end
        2'd2:    begin side_val = {1'b0, dss[3:2]}; delay_field = {3'b000, dss[1:0]};end
        default: begin side_val = dss[3:1];         delay_field = {4'b0000, dss[0]}; end
      endcase
    end else begin
      side_en = (side_count != 2'd0);
      case (side_count)
        2'd0: begin side_val = 3'd0;             delay_field = dss;               end
        2'd1: begin side_val = {2'b00, dss[4]};  delay_field = {1'b0, dss[3:0]};  end
        2'd2: begin side_val = {1'b0, dss[4:3]}; delay_field = {2'b00, dss[2:0]}; end
        2'd3: begin side_val = dss[4:2];         delay_field = {3'b000, dss[1:0]};end
      endcase
    end
  end

  wire [3:0] eff_n     = (ci[4:0] == 5'd0 || ci[4:0] > 5'd8) ? 4'd8 : ci[3:0];
  wire [3:0] pull_eff  = (pull_thresh == 4'd0) ? 4'd8 : pull_thresh;
  wire [3:0] push_eff  = (push_thresh == 4'd0) ? 4'd8 : push_thresh;
  wire       osr_empty = (osr_cnt >= pull_eff);

  // 8 input pins starting at in_base
  reg [7:0] pins_in8;
  integer pi;
  always @* begin
    for (pi = 0; pi < 8; pi = pi + 1)
      pins_in8[pi] = gpio_in[({28'd0, in_base} + pi) % 16];
  end

  wire attempt = tick && (delay_q == 5'd0);

  // ------------------------------------------------------------------
  // combinational execute
  // ------------------------------------------------------------------
  reg        do_stall;
  reg        x_we, y_we, osr_we, isr_we, osr_cnt_we, isr_cnt_we;
  reg [7:0]  x_n, y_n, osr_n, isr_n;
  reg [3:0]  osr_cnt_n, isr_cnt_n;
  reg        jump_en;
  reg [4:0]  jump_addr;
  reg        exec_set;
  reg [7:0]  exec_byte_n;
  reg        irq_sent_set, irq_sent_clr;

  reg [15:0] op_pin_mask, op_pin_data;

  // scratch
  reg [7:0]  in_val, mov_val, out_bits, isr_shift, osr_shifted;
  reg [4:0]  cnt_sum;
  reg        cond_true, wait_val, pp_eff;
  reg [7:0]  osr_eff;
  reg [3:0]  osr_cnt_eff;
  reg        need_refill;

  always @* begin
    do_stall     = 1'b0;
    x_we         = 1'b0;  x_n = 8'd0;
    y_we         = 1'b0;  y_n = 8'd0;
    osr_we       = 1'b0;  osr_n = 8'd0;
    isr_we       = 1'b0;  isr_n = 8'd0;
    osr_cnt_we   = 1'b0;  osr_cnt_n = 4'd0;
    isr_cnt_we   = 1'b0;  isr_cnt_n = 4'd0;
    jump_en      = 1'b0;  jump_addr = 5'd0;
    exec_set     = 1'b0;  exec_byte_n = 8'd0;
    irq_sent_set = 1'b0;  irq_sent_clr = 1'b0;
    tx_pop       = 1'b0;
    rx_push      = 1'b0;  rx_wdata = 8'd0;
    irq_set      = 4'd0;
    irq_clr      = 4'd0;
    op_pin_mask  = 16'd0; op_pin_data = 16'd0;
    dir_wr_mask  = 16'd0; dir_wr_data = 16'd0;
    in_val       = 8'd0;  mov_val = 8'd0; out_bits = 8'd0;
    isr_shift    = 8'd0;  osr_shifted = 8'd0;
    cnt_sum      = 5'd0;
    cond_true    = 1'b0;  wait_val = 1'b0; pp_eff = 1'b0;
    osr_eff      = 8'd0;  osr_cnt_eff = 4'd0;
    need_refill  = 1'b0;

    case (opc)
      // ------------------------------------------------ JMP
      3'b000: begin
        case (ci[7:5])
          3'd0: cond_true = 1'b1;
          3'd1: cond_true = (x == 8'd0);
          3'd2: begin cond_true = (x != 8'd0); x_we = 1'b1; x_n = x - 8'd1; end
          3'd3: cond_true = (y == 8'd0);
          3'd4: begin cond_true = (y != 8'd0); y_we = 1'b1; y_n = y - 8'd1; end
          3'd5: cond_true = (x != y);
          3'd6: cond_true = gpio_in[jmp_pin];
          3'd7: cond_true = !osr_empty;
        endcase
        jump_en   = cond_true;
        jump_addr = ci[4:0];
      end

      // ------------------------------------------------ WAIT
      3'b001: begin
        case (ci[6:5])
          2'd0: wait_val = gpio_in[ci[3:0]];
          2'd1: wait_val = gpio_in[({28'd0, in_base} + {27'd0, ci[3:0]}) % 16];
          2'd2: wait_val = irq_flags[ci[1:0]];
          default: wait_val = 1'b1;
        endcase
        do_stall = (wait_val != ci[7]);
        // wait-for-set on IRQ clears the flag when the wait completes
        if (attempt && !do_stall && ci[6:5] == 2'd2 && ci[7])
          irq_clr[ci[1:0]] = 1'b1;
      end

      // ------------------------------------------------ IN
      3'b010: begin
        case (ci[7:5])
          3'd0: in_val = pins_in8;
          3'd1: in_val = x;
          3'd2: in_val = y;
          3'd3: in_val = 8'd0;
          3'd6: in_val = isr;
          3'd7: in_val = osr;
          default: in_val = 8'd0;
        endcase
        in_val = in_val & bitmask8(eff_n);
        if (in_shift_right)
          isr_shift = (isr >> eff_n) | (in_val << (4'd8 - eff_n));
        else
          isr_shift = (isr << eff_n) | in_val;
        cnt_sum   = {1'b0, isr_cnt} + {1'b0, eff_n};
        isr_cnt_n = (cnt_sum > 5'd8) ? 4'd8 : cnt_sum[3:0];
        isr_we     = 1'b1;
        isr_cnt_we = 1'b1;
        isr_n      = isr_shift;
        if (autopush && (isr_cnt_n >= push_eff)) begin
          if (rx_full) begin
            do_stall   = 1'b1;
            isr_we     = 1'b0;
            isr_cnt_we = 1'b0;
          end else begin
            rx_push   = attempt;
            rx_wdata  = isr_shift;
            isr_n     = 8'd0;
            isr_cnt_n = 4'd0;
          end
        end
      end

      // ------------------------------------------------ OUT
      3'b011: begin
        need_refill = autopull && osr_empty;
        if (need_refill && tx_empty) begin
          do_stall = 1'b1;
        end else begin
          osr_eff     = need_refill ? tx_rdata : osr;
          osr_cnt_eff = need_refill ? 4'd0     : osr_cnt;
          tx_pop      = need_refill && attempt;
          if (out_shift_right) begin
            out_bits    = osr_eff & bitmask8(eff_n);
            osr_shifted = osr_eff >> eff_n;
          end else begin
            out_bits    = osr_eff >> (4'd8 - eff_n);
            osr_shifted = osr_eff << eff_n;
          end
          osr_we    = 1'b1;
          osr_n     = osr_shifted;
          cnt_sum   = {1'b0, osr_cnt_eff} + {1'b0, eff_n};
          osr_cnt_we = 1'b1;
          osr_cnt_n  = (cnt_sum > 5'd8) ? 4'd8 : cnt_sum[3:0];
          case (ci[7:5])
            3'd0: begin // PINS
              op_pin_mask = pin_mask(out_base, out_count);
              op_pin_data = pin_spread(out_base, out_count, out_bits);
            end
            3'd1: begin x_we = 1'b1; x_n = out_bits; end
            3'd2: begin y_we = 1'b1; y_n = out_bits; end
            3'd3: ; // NULL
            3'd4: begin // PINDIRS
              dir_wr_mask = attempt ? pin_mask(out_base, out_count) : 16'd0;
              dir_wr_data = pin_spread(out_base, out_count, out_bits);
            end
            3'd5: begin jump_en = 1'b1; jump_addr = out_bits[4:0]; end
            3'd6: begin isr_we = 1'b1; isr_n = out_bits;
                        isr_cnt_we = 1'b1; isr_cnt_n = eff_n; end
            default: ;
          endcase
        end
      end

      // ------------------------------------------------ PUSH / PULL
      3'b100: begin
        if (ci[7]) begin // PULL
          pp_eff = !ci[6] || osr_empty; // ifempty
          if (pp_eff) begin
            if (tx_empty) begin
              if (ci[5]) do_stall = 1'b1;
              else begin // non-blocking pull on empty: OSR <= X
                osr_we = 1'b1; osr_n = x;
                osr_cnt_we = 1'b1; osr_cnt_n = 4'd0;
              end
            end else begin
              tx_pop = attempt;
              osr_we = 1'b1; osr_n = tx_rdata;
              osr_cnt_we = 1'b1; osr_cnt_n = 4'd0;
            end
          end
        end else begin // PUSH
          pp_eff = !ci[6] || (isr_cnt >= push_eff); // iffull
          if (pp_eff) begin
            if (rx_full) begin
              if (ci[5]) do_stall = 1'b1;
              else begin // non-blocking push on full: data dropped
                isr_we = 1'b1; isr_n = 8'd0;
                isr_cnt_we = 1'b1; isr_cnt_n = 4'd0;
              end
            end else begin
              rx_push = attempt;
              rx_wdata = isr;
              isr_we = 1'b1; isr_n = 8'd0;
              isr_cnt_we = 1'b1; isr_cnt_n = 4'd0;
            end
          end
        end
      end

      // ------------------------------------------------ MOV
      3'b101: begin
        case (ci[2:0])
          3'd0: mov_val = pins_in8;
          3'd1: mov_val = x;
          3'd2: mov_val = y;
          3'd3: mov_val = 8'd0;
          3'd5: mov_val = status_sel ? 8'hFF : 8'h00; // STATUS
          3'd6: mov_val = isr;
          3'd7: mov_val = osr;
          default: mov_val = 8'd0;
        endcase
        case (ci[4:3])
          2'd1: mov_val = ~mov_val;
          2'd2: mov_val = rev8(mov_val);
          default: ;
        endcase
        case (ci[7:5])
          3'd0: begin // PINS
            op_pin_mask = pin_mask(out_base, out_count);
            op_pin_data = pin_spread(out_base, out_count, mov_val);
          end
          3'd1: begin x_we = 1'b1; x_n = mov_val; end
          3'd2: begin y_we = 1'b1; y_n = mov_val; end
          3'd4: begin exec_set = 1'b1; exec_byte_n = mov_val; end
          3'd5: begin jump_en = 1'b1; jump_addr = mov_val[4:0]; end
          3'd6: begin isr_we = 1'b1; isr_n = mov_val;
                      isr_cnt_we = 1'b1; isr_cnt_n = 4'd0; end
          3'd7: begin osr_we = 1'b1; osr_n = mov_val;
                      osr_cnt_we = 1'b1; osr_cnt_n = 4'd0; end
          default: ;
        endcase
      end

      // ------------------------------------------------ IRQ
      3'b110: begin
        if (ci[6]) begin
          if (attempt) irq_clr[ci[1:0]] = 1'b1;
        end else if (ci[5]) begin // set and wait for clear
          if (!irq_sent) begin
            if (attempt) irq_set[ci[1:0]] = 1'b1;
            irq_sent_set = 1'b1;
            do_stall = 1'b1;
          end else if (irq_flags[ci[1:0]]) begin
            do_stall = 1'b1;
          end else begin
            irq_sent_clr = 1'b1;
          end
        end else begin
          if (attempt) irq_set[ci[1:0]] = 1'b1;
        end
      end

      // ------------------------------------------------ SET
      3'b111: begin
        case (ci[7:5])
          3'd0: begin // PINS
            op_pin_mask = pin_mask(set_base, {1'b0, set_count});
            op_pin_data = pin_spread(set_base, {1'b0, set_count}, {3'd0, ci[4:0]});
          end
          3'd1: begin x_we = 1'b1; x_n = {3'd0, ci[4:0]}; end
          3'd2: begin y_we = 1'b1; y_n = {3'd0, ci[4:0]}; end
          3'd4: begin // PINDIRS
            dir_wr_mask = attempt ? pin_mask(set_base, {1'b0, set_count}) : 16'd0;
            dir_wr_data = pin_spread(set_base, {1'b0, set_count}, {3'd0, ci[4:0]});
          end
          default: ;
        endcase
      end
    endcase

    // qualify strobes that fire only on successful execution
    if (!attempt || do_stall) begin
      op_pin_mask = 16'd0;
      dir_wr_mask = 16'd0;
      tx_pop      = 1'b0;
      rx_push     = 1'b0;
    end

    // side-set: applies on every attempt, even when stalled; wins conflicts
    pin_wr_mask = op_pin_mask;
    pin_wr_data = op_pin_data;
    if (attempt && side_en) begin
      if (side_pindir) begin
        dir_wr_mask = dir_wr_mask | pin_mask(side_base, {2'd0, side_count});
        dir_wr_data = (dir_wr_data & ~pin_mask(side_base, {2'd0, side_count}))
                    | pin_spread(side_base, {2'd0, side_count}, {5'd0, side_val});
      end else begin
        pin_wr_mask = op_pin_mask | pin_mask(side_base, {2'd0, side_count});
        pin_wr_data = (op_pin_data & ~pin_mask(side_base, {2'd0, side_count}))
                    | pin_spread(side_base, {2'd0, side_count}, {5'd0, side_val});
      end
    end
  end

  // ------------------------------------------------------------------
  // sequential state update
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n || restart) begin
      // restart begins execution at wrap_bottom (the program origin),
      // so the two SMs can run different programs in the shared memory
      pc         <= rst_n ? wrap_bottom : 5'd0;
      x          <= 8'd0;
      y          <= 8'd0;
      osr        <= 8'd0;
      isr        <= 8'd0;
      osr_cnt    <= 4'd8; // empty
      isr_cnt    <= 4'd0;
      delay_q    <= 5'd0;
      exec_valid <= 1'b0;
      exec_byte  <= 8'd0;
      irq_sent   <= 1'b0;
      stalled    <= 1'b0;
    end else if (tick) begin
      if (delay_q != 5'd0) begin
        delay_q <= delay_q - 5'd1;
      end else begin
        stalled <= do_stall;
        if (do_stall) begin
          if (irq_sent_set) irq_sent <= 1'b1;
        end else begin
          if (x_we)       x       <= x_n;
          if (y_we)       y       <= y_n;
          if (osr_we)     osr     <= osr_n;
          if (isr_we)     isr     <= isr_n;
          if (osr_cnt_we) osr_cnt <= osr_cnt_n;
          if (isr_cnt_we) isr_cnt <= isr_cnt_n;
          if (irq_sent_clr) irq_sent <= 1'b0;
          if (exec_set) begin
            exec_valid <= 1'b1;
            exec_byte  <= exec_byte_n;
          end else if (exec_valid) begin
            exec_valid <= 1'b0;
          end
          delay_q <= delay_field;
          if (jump_en)          pc <= jump_addr;
          else if (!exec_valid) pc <= (pc == wrap_top) ? wrap_bottom : pc + 5'd1;
        end
      end
    end
  end

endmodule
