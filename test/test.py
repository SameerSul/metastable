# Metastable protocol emulator - cocotb tests
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0

import collections
import json
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from metastable import (
    SPI_HALF, SpiHost, uio_loopback, I2CSlave, WS2812Decoder, UsbLsDecoder,
    usb_crc16, Ps2Host, nec_transmit, side,
    SpiMonitor, I2CTarget, I2C_SPEC_NS,
    JMP, WAIT, IN_, OUT, PUSH, PULL, MOV, IRQ, SET,
    SRC_PINS, SRC_X, SRC_Y, SRC_NULL, SRC_STATUS, SRC_OSR,
    ODST_PINS, ODST_NULL, ODST_PC, ODST_ISR,
    MDST_X, MDST_EXEC, MDST_ISR, M_INV, M_REV,
    SDST_PINS, SDST_PINDIRS, SDST_X, SDST_Y,
    C_XDEC, C_YDEC, C_XNEY, C_PIN, C_NOTOSRE, W_GPIO, W_IRQ,
    R_CTRL, R_FSTAT, R_IRQ, R_IRQ_MASK, R_GPIO_IN_L, R_GPIO_IN_H, R_PC0,
    R_FLEVEL0, SM,
    CLKDIV_INT_L, CLKDIV_FRAC, WRAP_TOP, WRAP_BOTTOM, SHIFTCTRL, THRESH,
    PIN_OUT, PIN_SET, PIN_IN, PIN_SIDE, JMP_PIN,
    AUTOPULL, AUTOPUSH, SIDE_OPT,
    TXF, RXF, STATUS_CFG, STATUS_RX,
)
from golden import random_case, random_case_pins
import capture_replay
import i2c_read
import i2c_write
import manchester
import nec_ir_receive
import ps2_device
import spi_modes
import two_protocols
import uart_loopback
import usb_ls_tx
import ws2812


async def configure(spi, writes):
    """Apply the register writes of an example: a list of (address, value)."""
    for addr, value in writes:
        await spi.write(addr, value)


# How many seeds the two constrained-random tests run. A push runs 12 and
# 8. The scheduled job runs 120 and 80, and then checks the coverage
# table (cosim_coverage.py).
ARCH_SEEDS = int(os.environ.get("COSIM_ARCH_SEEDS", "12"))
PIN_SEEDS = int(os.environ.get("COSIM_PIN_SEEDS", "8"))


def save_coverage(name, seeds, resampled, steps, cells):
    """What one constrained-random test executed, for cosim_coverage.py."""
    with open("cosim_coverage_%s.json" % name, "w") as f:
        json.dump({"seeds": seeds, "resampled": resampled, "steps": steps,
                   "cells": dict(sorted(cells.items()))}, f, indent=1)
        f.write("\n")


async def setup(dut):
    dut._log.info("start")
    clock = Clock(dut.clk, 20, unit="ns")  # 50 MHz
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0x02  # CS_N high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    return SpiHost(dut)


@cocotb.test()
async def test_registers(dut):
    """SPI register and instruction memory read/write."""
    spi = await setup(dut)

    # instruction memory readback (8 bytes = 4 instructions)
    pattern = [0x11, 0x22, 0x33, 0x44, 0xA5, 0x5A, 0xF0, 0x0F]
    await spi.write(0x00, pattern)
    got = await spi.read(0x00, 8)
    assert got == pattern, f"imem readback {got} != {pattern}"

    # config register readback
    await spi.write(SM(0) + CLKDIV_INT_L, 0x37)
    got = await spi.read(SM(0) + CLKDIV_INT_L, 1)
    assert got == [0x37]

    # FSTAT: all FIFOs empty, none full
    got = await spi.read(R_FSTAT, 1)
    assert got == [0x55], f"FSTAT {got[0]:#04x} != 0x55"

    # GPIO input readback: drive input-only pins GPIO8 (ui[3]) and GPIO10 (ui[5])
    dut.ui_in.value = (1 << 3) | (1 << 5) | 0x02
    await ClockCycles(dut.clk, 5)
    got = await spi.read(R_GPIO_IN_H, 1)
    assert got == [0x05], f"GPIO_IN_H {got[0]:#04x} != 0x05"
    dut.ui_in.value = 0x02


@cocotb.test()
async def test_uart_loopback(dut):
    """SM0 transmits UART frames on GPIO0; SM1 receives them on the same pin."""
    spi = await setup(dut)
    cocotb.start_soon(uio_loopback(dut))

    # the programs and settings of sw/examples/uart_loopback.py: 8N1, LSB
    # first, 8 ticks a bit, transmitter on SM0 and receiver on SM1
    await spi.load_program(0, uart_loopback.TX_PROG)
    await spi.load_program(uart_loopback.RX0, uart_loopback.RX_PROG)
    await configure(spi, uart_loopback.config(clkdiv=2))

    # start SM0 first so the line is idle high before the receiver arms
    await spi.write(R_CTRL, 0x11)  # SM0 enable + restart
    await ClockCycles(dut.clk, 100)
    await spi.write(R_CTRL, 0x23)  # keep SM0, SM1 enable + restart

    payload = [0xA5, 0x3C, 0x00, 0xFF]
    for b in payload:
        await spi.write(SM(0) + TXF, b)

    received = []
    for _ in range(300):
        fstat = await spi.read(R_FSTAT, 1)
        if not fstat[0] & 0x40:  # SM1 RX FIFO non-empty
            received += await spi.read(SM(1) + RXF, 1)
            if len(received) == len(payload):
                break
        await ClockCycles(dut.clk, 50)

    assert received == payload, f"UART loopback {received} != {payload}"


@cocotb.test()
async def test_spi_master_slave(dut):
    """SM0 is a mode-0 SPI master (optional side-set SCK), SM1 the slave.

    GPIO0 = SCK, GPIO1 = MOSI, GPIO2 = MISO. Full duplex: master TX bytes
    arrive in the slave's RX FIFO and vice versa, MSB first.
    """
    spi = await setup(dut)
    cocotb.start_soon(uio_loopback(dut))

    OPT = True  # per-instruction side-set: only OUT/IN/JMP touch SCK
    m_prog = [
        SET(SDST_PINDIRS, 3),                            # 0: SCK+MOSI outputs
        SET(SDST_X, 7),                                  # 1: bit counter
        OUT(ODST_PINS, 1, side=side(0, 1, OPT), delay=1),  # 2: MOSI, SCK low
        IN_(SRC_PINS, 1, side=side(1, 1, OPT)),          # 3: sample on rise
        JMP(2, C_XDEC, side=side(1, 1, OPT)),            # 4: SCK high 2nd half
    ]
    # slave: event-driven off SCK edges, runs at div 1 (4x master tick rate)
    S0 = 8
    s_prog = [
        SET(SDST_PINDIRS, 1),    # 8: MISO output
        OUT(ODST_PINS, 1),       # 9: drive MISO while SCK low
        WAIT(1, W_GPIO, 0),      # 10: SCK rise
        IN_(SRC_PINS, 1),        # 11: sample MOSI
        WAIT(0, W_GPIO, 0),      # 12: SCK fall
    ]

    await spi.load_program(0, m_prog)
    await spi.load_program(S0, s_prog)

    # SM0 master: div 8 (SCK low phase must exceed the slave's ~11 clk
    # response latency: pin router + pad loopback + two 2FF synchronizers)
    await spi.write(SM(0) + CLKDIV_INT_L, 8)
    await spi.write(SM(0) + WRAP_TOP, 4)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(SM(0) + SHIFTCTRL, AUTOPULL | AUTOPUSH)
    await spi.write(SM(0) + PIN_OUT, 0x11)             # MOSI: base 1, count 1
    await spi.write(SM(0) + PIN_SET, 0x20)             # base 0, count 2
    await spi.write(SM(0) + PIN_IN, 2)                 # MISO
    await spi.write(SM(0) + PIN_SIDE, SIDE_OPT | 0x10)  # SCK: base 0, count 1

    # SM1 slave: div 1, MSB first, autopull+autopush
    await spi.write(SM(1) + CLKDIV_INT_L, 1)
    await spi.write(SM(1) + WRAP_TOP, 12)
    await spi.write(SM(1) + WRAP_BOTTOM, S0)
    await spi.write(SM(1) + SHIFTCTRL, AUTOPULL | AUTOPUSH)
    await spi.write(SM(1) + PIN_OUT, 0x12)             # MISO: base 2, count 1
    await spi.write(SM(1) + PIN_SET, 0x12)             # MISO dir
    await spi.write(SM(1) + PIN_IN, 1)                 # MOSI

    # start both SMs first (restart flushes FIFOs): they stall at OUT with
    # SCK idle low. Then load the slave's responses, then the master's data.
    await spi.write(R_CTRL, 0x33)
    mosi_bytes = [0x9E, 0x01, 0x55, 0xC3]
    miso_bytes = [0xEF, 0x40, 0xAA, 0x81]
    for b in miso_bytes:
        await spi.write(SM(1) + TXF, b)
    for b in mosi_bytes:
        await spi.write(SM(0) + TXF, b)

    m_rx, s_rx = [], []
    for _ in range(300):
        fstat = (await spi.read(R_FSTAT, 1))[0]
        if not fstat & 0x04:  # SM0 RX non-empty
            m_rx += await spi.read(SM(0) + RXF, 1)
        if not fstat & 0x40:  # SM1 RX non-empty
            s_rx += await spi.read(SM(1) + RXF, 1)
        if len(m_rx) == 4 and len(s_rx) == 4:
            break
        await ClockCycles(dut.clk, 20)

    assert s_rx == mosi_bytes, f"slave RX {s_rx} != {mosi_bytes}"
    assert m_rx == miso_bytes, f"master RX {m_rx} != {miso_bytes}"


@cocotb.test()
async def test_i2c_master(dut):
    """SM0 bit-bangs an open-drain I2C master write.

    SDA = GPIO0 (OUT/SET pindirs), SCL = GPIO1 (optional side-set routed
    to pin directions). Pin output values stay latched at 0, so dir=1
    drives low and dir=0 releases to the pull-up: true open drain. A
    Python bus model supplies the pull-ups and a slave that ACKs each
    byte; the ACK bits come back to the host through the RX FIFO.
    """
    spi = await setup(dut)
    slave = I2CSlave(dut)
    cocotb.start_soon(slave.run())

    # the program and settings of sw/examples/i2c_write.py: two bytes
    await spi.load_program(0, i2c_write.PROG)
    await configure(spi, i2c_write.config(clkdiv=8))

    await spi.write(R_CTRL, 0x11)   # SM0 enable + restart (flushes FIFOs)
    wire_bytes = [0xA4, 0x57]       # address 0x52 + W, one data byte
    for b in wire_bytes:
        await spi.write(SM(0) + TXF, ~b & 0xFF)  # dir=1 pulls the line low

    acks = []
    for _ in range(300):
        fstat = (await spi.read(R_FSTAT, 1))[0]
        if not fstat & 0x04:  # SM0 RX non-empty
            acks = await spi.read(SM(0) + RXF, 1)
            break
        await ClockCycles(dut.clk, 50)

    assert slave.stopped, "no STOP condition seen"
    assert slave.bytes == wire_bytes, f"slave got {slave.bytes} != {wire_bytes}"
    assert acks == [0x00], f"ACK bits {acks[0] if acks else None} != 0x00"


async def drain_rx(spi, dut, sm, limit, polls=80):
    """Collect up to `limit` RX FIFO bytes from state machine `sm`."""
    got = []
    empty_bit = 0x04 << (4 * sm)
    for _ in range(polls):
        fstat = (await spi.read(R_FSTAT, 1))[0]
        if not fstat & empty_bit:
            got += await spi.read(SM(sm) + RXF, 1)
            if len(got) >= limit:
                break
        else:
            await ClockCycles(dut.clk, 50)
    return got


@cocotb.test()
async def test_mov_ops(dut):
    """MOV invert / bit-reverse / STATUS source / EXEC computed jump."""
    spi = await setup(dut)

    b, t = 0x3A, 12
    prog = [
        PULL(block=1),                    # 0: OSR = b
        MOV(MDST_X, SRC_OSR),             # 1: X = b
        MOV(MDST_ISR, SRC_X, M_INV),      # 2:
        PUSH(block=1),                    # 3: push ~b
        MOV(MDST_ISR, SRC_X, M_REV),      # 4:
        PUSH(block=1),                    # 5: push rev(b)
        MOV(MDST_ISR, SRC_STATUS),        # 6: TX empty here -> 0xFF
        PUSH(block=1),                    # 7:
        PULL(block=1),                    # 8: OSR = t
        MOV(MDST_EXEC, SRC_OSR),          # 9: inject JMP t
        MOV(MDST_ISR, SRC_NULL),          # 10: skipped
        PUSH(block=1),                    # 11: skipped
        MOV(MDST_ISR, SRC_X),             # 12: EXEC lands here
        PUSH(block=1),                    # 13: push b
        JMP(14),                          # 14: park
    ]
    await spi.load_program(0, prog)
    await spi.write(SM(0) + CLKDIV_INT_L, 8)
    await spi.write(SM(0) + WRAP_TOP, 14)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)

    await spi.write(R_CTRL, 0x11)
    await spi.write(SM(0) + TXF, b)
    await spi.write(SM(0) + TXF, t)

    got = await drain_rx(spi, dut, 0, 4)
    exp = [~b & 0xFF, 0x5C, 0xFF, b]  # rev(0x3A) = 0x5C
    assert got == exp, f"MOV results {got} != {exp}"

    # second pass: STATUS reconfigured to RX level < 2; two pushes have
    # landed by the time MOV STATUS runs, so it must now read all-zeros
    await spi.write(SM(0) + STATUS_CFG, STATUS_RX | 2)
    await spi.write(R_CTRL, 0x11)
    await spi.write(SM(0) + TXF, b)
    await spi.write(SM(0) + TXF, t)
    got = await drain_rx(spi, dut, 0, 4)
    exp = [~b & 0xFF, 0x5C, 0x00, b]
    assert got == exp, f"MOV results (RX status) {got} != {exp}"


@cocotb.test()
async def test_irq_handshake(dut):
    """Inter-SM IRQ set/wait/clear handshake plus the HOST_IRQ pin."""
    spi = await setup(dut)

    sm0 = [
        IRQ(0, wait=1),          # 0: set flag0, stall until cleared
        WAIT(1, W_IRQ, 1),       # 1: wait for flag1, clear it
        IRQ(2),                  # 2: flag2 -> host interrupt
        SET(SDST_Y, 9),          # 3:
        MOV(MDST_ISR, SRC_Y),    # 4:
        PUSH(block=1),           # 5: completion marker
        JMP(6),                  # 6: park
    ]
    S1 = 16
    sm1 = [
        WAIT(1, W_IRQ, 0),       # 16: sees flag0, clears it (releases SM0)
        IRQ(1),                  # 17: handshake back
        JMP(18),                 # 18: park
    ]
    await spi.load_program(0, sm0)
    await spi.load_program(S1, sm1)
    await spi.write(SM(0) + WRAP_TOP, 6)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(SM(1) + WRAP_TOP, 18)
    await spi.write(SM(1) + WRAP_BOTTOM, S1)
    await spi.write(R_IRQ_MASK, 0x04)

    await spi.write(R_CTRL, 0x33)

    flags = 0
    for _ in range(50):
        flags = (await spi.read(R_IRQ, 1))[0]
        if flags & 0x04:
            break
        await ClockCycles(dut.clk, 20)
    assert flags & 0x04, f"IRQ2 never set (flags {flags:#04x})"
    assert flags & 0x03 == 0, f"handshake flags not cleared ({flags:#04x})"
    assert dut.uo_out.value.binstr[-2] == "1", "HOST_IRQ pin not asserted"

    await spi.write(R_IRQ, 0x04)  # W1C
    flags = (await spi.read(R_IRQ, 1))[0]
    assert flags == 0, f"IRQ flags {flags:#04x} != 0 after W1C"
    assert dut.uo_out.value.binstr[-2] == "0", "HOST_IRQ pin stuck high"

    got = await drain_rx(spi, dut, 0, 1)
    assert got == [9], f"marker {got} != [9]"


@cocotb.test()
async def test_jmp_variants(dut):
    """WAIT GPIO, JMP PIN / X!=Y / !OSRE, OUT ISR / NULL / PC."""
    spi = await setup(dut)

    d, t = 0xB7, 25
    prog = [
        WAIT(1, W_GPIO, 8),          # 0: host raises GPIO8 (ui[3])
        JMP(4, C_PIN),               # 1: taken (jmp_pin = GPIO8 high)
        SET(SDST_X, 31),             # 2: \ not-taken path
        JMP(5),                      # 3: /
        SET(SDST_X, 1),              # 4: taken path
        SET(SDST_Y, 1),              # 5:
        JMP(9, C_XNEY),              # 6: X==Y, not taken
        SET(SDST_Y, 3),              # 7:
        JMP(10, C_XNEY),             # 8: 1 != 3, taken
        SET(SDST_Y, 31),             # 9: skipped
        MOV(MDST_ISR, SRC_Y),        # 10:
        PUSH(block=1),               # 11: marker 3
        PULL(block=1),               # 12: OSR = d
        OUT(ODST_ISR, 4),            # 13: ISR = d[7:4] (shift left)
        JMP(16, C_NOTOSRE),          # 14: OSR half full, taken
        MOV(MDST_ISR, SRC_NULL),     # 15: skipped
        PUSH(block=1),               # 16: marker d >> 4
        OUT(ODST_NULL, 4),           # 17: drain OSR
        JMP(20, C_NOTOSRE),          # 18: OSR empty, not taken
        SET(SDST_X, 9),              # 19: executed
        MOV(MDST_ISR, SRC_X),        # 20:
        PUSH(block=1),               # 21: marker 9
        PULL(block=1),               # 22: OSR = t
        OUT(ODST_PC, 8),             # 23: computed jump: PC = t
        PUSH(block=1),               # 24: skipped (would inject extra byte)
        MOV(MDST_ISR, SRC_X),        # 25: OUT PC lands here
        PUSH(block=1),               # 26: marker 9
        JMP(27),                     # 27: park
    ]
    await spi.load_program(0, prog)
    await spi.write(SM(0) + CLKDIV_INT_L, 4)
    await spi.write(SM(0) + WRAP_TOP, 27)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(SM(0) + SHIFTCTRL, 0)  # shift left, no auto
    await spi.write(SM(0) + JMP_PIN, 8)

    await spi.write(R_CTRL, 0x11)
    dut.ui_in.value = int(dut.ui_in.value) | (1 << 3)  # GPIO8 high
    await ClockCycles(dut.clk, 2)  # let the deposit land before more SPI
    await spi.write(SM(0) + TXF, d)
    await spi.write(SM(0) + TXF, t)

    got = await drain_rx(spi, dut, 0, 6, polls=60)
    exp = [3, d >> 4, 9, 9]
    assert got == exp, f"markers {got} != {exp}"


@cocotb.test()
async def test_fifo_semantics(dut):
    """Non-blocking PULL, PUSH iffull threshold, RX backpressure."""
    spi = await setup(dut)

    prog = [
        SET(SDST_X, 26),             # 0:
        PULL(block=0),               # 1: TX empty -> OSR = X
        MOV(MDST_ISR, SRC_OSR),      # 2:
        PUSH(block=1),               # 3: push 26
        SET(SDST_Y, 8),              # 4:
        MOV(MDST_ISR, SRC_Y),        # 5:
        PUSH(block=1),               # 6: push 8..0 (stalls on full RX)
        JMP(5, C_YDEC),              # 7:
        IN_(SRC_NULL, 2),            # 8: isr_cnt = 2
        PUSH(iffull=1, block=1),     # 9: below threshold 4 -> no-op
        IN_(SRC_NULL, 2),            # 10: isr_cnt = 4
        PUSH(iffull=1, block=1),     # 11: at threshold -> push 0
        SET(SDST_X, 30),             # 12:
        MOV(MDST_ISR, SRC_X),        # 13:
        PUSH(block=1),               # 14: push 30
        JMP(15),                     # 15: park
    ]
    await spi.load_program(0, prog)
    await spi.write(SM(0) + WRAP_TOP, 15)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(SM(0) + THRESH, 0x40)  # push threshold 4, pull 8

    await spi.write(R_CTRL, 0x11)

    # SM runs at div 1: RX (8 deep) must fill and stall the SM
    full = False
    for _ in range(20):
        fstat = (await spi.read(R_FSTAT, 1))[0]
        if fstat & 0x08:  # RX0 full
            full = True
            break
        await ClockCycles(dut.clk, 10)
    assert full, "RX FIFO never filled under backpressure"
    assert dut.uo_out.value.binstr[-6] == "1", "SM0 not stalled on full RX"
    flevel = (await spi.read(R_FLEVEL0, 1))[0]
    assert flevel == 0x80, f"FLEVEL0 {flevel:#04x} != 0x80 (RX 8, TX 0)"

    got = await drain_rx(spi, dut, 0, 12)
    exp = [26, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 30]
    assert got == exp, f"FIFO sequence {got} != {exp}"


@cocotb.test()
async def test_clkdiv_restart(dut):
    """Fractional clock divider rate and mid-run restart semantics."""
    spi = await setup(dut)

    prog = [
        SET(SDST_PINDIRS, 1),  # 0: GPIO0 output
        SET(SDST_PINS, 1),     # 1:
        SET(SDST_PINS, 0),     # 2: 3-tick period
    ]
    await spi.load_program(0, prog)
    await spi.write(SM(0) + WRAP_TOP, 2)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(SM(0) + PIN_SET, 0x10)  # GPIO0: base 0, count 1
    await spi.write(SM(0) + CLKDIV_INT_L, 2)
    await spi.write(SM(0) + CLKDIV_FRAC, 128)  # divide by 2.5

    await spi.write(R_CTRL, 0x11)

    # 20 pin periods = 20 * 3 ticks * 2.5 clk = 150 clk
    edges, cycles, c1, c21, prev = 0, 0, None, None, 0
    for _ in range(4000):
        await ClockCycles(dut.clk, 1)
        cycles += 1
        cur = int(dut.uio_out.value) & 1
        if cur and not prev:
            edges += 1
            if edges == 1:
                c1 = cycles
            elif edges == 21:
                c21 = cycles
                break
        prev = cur
    assert c21 is not None, f"only {edges} edges seen"
    n = c21 - c1
    assert abs(n - 150) <= 4, f"20 periods took {n} clk, expected ~150"

    # mid-run restart: TX flushed, PC back at wrap_bottom
    await spi.write(SM(0) + TXF, 0x77)
    fstat = (await spi.read(R_FSTAT, 1))[0]
    assert not fstat & 0x01, "TXF write did not land"
    await spi.write(R_CTRL, 0x10)  # restart + disable
    fstat = (await spi.read(R_FSTAT, 1))[0]
    assert fstat & 0x01, "restart did not flush TX FIFO"
    pc0 = (await spi.read(R_PC0, 1))[0]
    assert pc0 == 0, f"PC0 {pc0} != wrap_bottom after restart"


@cocotb.test()
async def test_manchester(dut):
    """SM0 transmits IEEE 802.3 Manchester on GPIO0; SM1 decodes it.

    Each bit is 8 SM ticks with a guaranteed mid-bit transition ('1' =
    rising, '0' = falling). The receiver locks onto the first mid-bit
    rising edge of an 0xAA preamble, then samples the second half of
    every bit at the transmitter's rate (same clock, zero drift).
    """
    spi = await setup(dut)
    cocotb.start_soon(uio_loopback(dut))

    # the programs and settings of sw/examples/manchester.py. At divider
    # 8 one byte on the wire (512 clk) outlasts a host FIFO write.
    await spi.load_program(0, manchester.TX_PROG)
    await spi.load_program(manchester.RX0, manchester.RX_PROG)
    await configure(spi, manchester.config(clkdiv=8))

    await spi.write(R_CTRL, 0x33)
    payload = [0x0F, 0x96, 0x55, 0xC3]
    for b in [manchester.PREAMBLE] + payload:  # preamble syncs the receiver
        await spi.write(SM(0) + TXF, b)

    # the receiver free-runs after the frame, so only the first bytes count
    got = await drain_rx(spi, dut, 1, 5)
    exp = [manchester.PREAMBLE] + payload
    assert got == exp, f"Manchester RX {got} != {exp}"


@cocotb.test()
async def test_ws2812(dut):
    """SM0 drives WS2812 (NeoPixel) data on GPIO0: 800 kHz, GRB MSB first.

    Fractional divider 6+64/256 gives an 8 MHz tick (125 ns); 10 ticks per
    bit. '0' = 3 ticks high / 7 low, '1' = 7 high / 3 low -- all four pulse
    widths inside the WS2812B datasheet windows. A pulse-width decoder model
    checks every pulse and reassembles the frame.
    """
    spi = await setup(dut)
    cocotb.start_soon(uio_loopback(dut))
    dec = WS2812Decoder(dut)
    cocotb.start_soon(dec.run())

    # the program and settings of sw/examples/ws2812.py
    await spi.load_program(0, ws2812.PROG)
    await configure(spi, ws2812.config())

    await spi.write(R_CTRL, 0x11)

    # two LEDs, GRB order: red then an arbitrary colour
    payload = [0x00, 0xFF, 0x00, 0x30, 0x81, 0xC3]
    await spi.write(SM(0) + TXF, payload)  # one burst; address holds on FIFO

    # 48 bits x 1.25 us = 60 us, plus the reset gap the decoder needs
    await ClockCycles(dut.clk, 5000)

    assert not dec.bad_pulses, f"out-of-spec pulses (ns): {dec.bad_pulses}"
    assert dec.frames, "no frame latched"
    assert dec.frames[0] == payload, f"WS2812 {dec.frames[0]} != {payload}"


@cocotb.test()
async def test_usb_ls_tx(dut):
    """SM0 transmits a USB low-speed DATA0 packet on D+/D- (GPIO0/1).

    Divider 4+43/256 gives 8 ticks per bit at 1.5 MHz +0.03%. The host
    pre-computes sync, NRZI, bit stuffing, CRC16 and EOP, packing four
    J/K/SE0 symbols per FIFO byte; the SM is just OUT PINS,2 every bit.
    Streaming over SPI at SCK = clk/8 outruns the 2.67 us/byte line
    consumption, so the 8-deep FIFO never underruns mid-packet. An
    independent decoder model samples the bus at bit centers, NRZI
    decodes, destuffs, and checks sync, PID, payload, CRC and EOP.
    """
    spi = await setup(dut)
    dec = UsbLsDecoder(dut)
    cocotb.start_soon(dec.run())

    # the program and settings of sw/examples/usb_ls_tx.py
    await spi.load_program(0, usb_ls_tx.PROG)
    await configure(spi, usb_ls_tx.config())
    await spi.write(R_CTRL, 0x11)

    payload = [0xFF, 0x3F, 0x00, 0xA5]  # 0xFF forces a stuffed bit
    pid = usb_ls_tx.PID_DATA0
    stream = usb_ls_tx.packet_bytes(pid, payload)

    # burst-fill the FIFO, then stream the rest at SCK = clk/8: one
    # 2-byte write is ~140 clk vs 133 clk consumed per symbol byte
    fast = SpiHost(dut, half=4)
    await fast.write(SM(0) + TXF, stream[:8])
    for b in stream[8:]:
        await fast.write(SM(0) + TXF, b)

    await ClockCycles(dut.clk, 3000)  # drain + EOP + idle

    data, wire_bits, bit_ns = dec.decode()
    crc = usb_crc16(payload)
    exp = [pid] + payload + [crc & 0xFF, (crc >> 8) & 0xFF]
    assert data == exp, f"USB LS packet {data} != {exp}"
    assert wire_bits > 8 + 8 * len(exp), "bit stuffing never happened"
    ppm = (bit_ns / (1e9 / 1.5e6) - 1) * 1e6
    dut._log.info(f"USB LS bit period {bit_ns:.1f} ns ({ppm:+.0f} ppm)")
    assert abs(ppm) < 15000, f"bit rate {ppm:+.0f} ppm outside USB LS +-1.5%"

    # dump the bus edges so docs/usb_wave.py can render the datasheet
    # figure from the actual simulated waveform
    import json
    with open("usb_edges.json", "w") as f:
        json.dump({"edges": dec.edges, "t_bit": dec.T_BIT_NS}, f)


@cocotb.test()
async def test_random_cosim(dut):
    """Constrained-random ISA verification against a golden model.

    Each seed generates a random program over the architectural subset
    (shifts, FIFOs, scratch ops, flow control, IRQ flags, WAIT on a
    flag, computed jumps, MOV EXEC, delays) plus random
    SHIFTCTRL/THRESH/STATUS_CFG and a random TX FIFO preload, runs it to
    its park point in both the golden model (test/golden.py) and the
    silicon RTL, then compares PC, IRQ flags, and the exact RX FIFO
    contents (which fold in X/Y through the dump epilogue).
    """
    spi = await setup(dut)
    cells, resampled, total_steps = collections.Counter(), 0, 0
    for seed in range(ARCH_SEEDS):
        prog, cfg, tx, g, steps, used_seed = random_case(seed)

        # halt SM0 and reset its state; restart also flushes the FIFOs
        await spi.write(SM(0) + WRAP_BOTTOM, 0)
        await spi.write(SM(0) + WRAP_TOP, 31)
        await spi.write(R_CTRL, 0x10)
        await spi.load_program(0, prog)
        await spi.write(SM(0) + CLKDIV_INT_L, 1)
        await spi.write(SM(0) + CLKDIV_FRAC, 0)
        await spi.write(SM(0) + SHIFTCTRL, cfg["shiftctrl"])
        await spi.write(SM(0) + THRESH, cfg["thresh"])
        await spi.write(SM(0) + STATUS_CFG, cfg["status_cfg"])
        if tx:
            await spi.write(SM(0) + TXF, tx)  # preload, then enable w/o restart
        await spi.write(R_CTRL, 0x01)

        # one tick a clock cycle: the model counts the ticks it needs
        await ClockCycles(dut.clk, g.ticks + 200)

        # freeze before inspecting: draining RX could unstall a blocked PUSH
        await spi.write(R_CTRL, 0x00)
        pc = (await spi.read(R_PC0, 1))[0]
        irq = (await spi.read(R_IRQ, 1))[0]
        rx = []
        while len(rx) <= 8 and not (await spi.read(R_FSTAT, 1))[0] & 0x04:
            rx += await spi.read(SM(0) + RXF, 1)

        exp_irq = sum(b << i for i, b in enumerate(g.irq))
        assert pc == g.pc, f"seed {used_seed}: PC {pc} != {g.pc}"
        assert rx == g.rx, f"seed {used_seed}: RX {rx} != {g.rx}"
        assert irq == exp_irq, f"seed {used_seed}: IRQ {irq:#x} != {exp_irq:#x}"
        await spi.write(R_IRQ, 0x0F)  # W1C for the next seed
        cells.update(g.cov)  # counted only once the hardware agrees
        resampled += used_seed != seed
        total_steps += steps
        dut._log.info(
            f"cosim seed {used_seed}: {steps} steps, pc={pc}, "
            f"rx={len(rx)} bytes, irq={irq:#x} OK")
    save_coverage("arch", ARCH_SEEDS, resampled, total_steps, cells)


@cocotb.test()
async def test_concurrent_protocols(dut):
    """Both SMs emulate different protocols at once: SM0 transmits a USB
    low-speed packet on GPIO0/1 (1.5 MHz, divider 4+43/256) while SM1
    drives a WS2812 frame on GPIO4 (800 kHz, divider 6+64/256). Each
    stream is checked by its own independent decoder model — the point
    is two unrelated bit rates from one chip at the same time.
    """
    spi = await setup(dut)
    usb = UsbLsDecoder(dut)
    led = WS2812Decoder(dut, pin=two_protocols.WS_PIN)
    cocotb.start_soon(usb.run())
    cocotb.start_soon(led.run())

    # the programs and settings of sw/examples/two_protocols.py: USB LS
    # on GPIO0/1 from SM0, WS2812 on GPIO4 from SM1
    await spi.load_program(two_protocols.USB_ORIGIN, two_protocols.USB_PROG)
    await spi.load_program(two_protocols.WS_ORIGIN, two_protocols.WS_PROG)
    await configure(spi, two_protocols.config())

    payload = [0x0F, 0x99]
    pid = usb_ls_tx.PID_DATA0
    stream = usb_ls_tx.packet_bytes(pid, payload)
    grb = [0x10, 0xFF, 0x08, 0x00, 0x40, 0xC3]  # two LEDs

    # WS2812 frame fits the 8-deep FIFO: preload it whole, then start
    # both SMs together and stream the USB symbols live
    await spi.write(R_CTRL, 0x30)  # both held in reset at wrap_bottom
    await spi.write(SM(1) + TXF, grb)
    await spi.write(SM(0) + TXF, stream[:8])
    await spi.write(R_CTRL, 0x03)  # both enabled, no restart
    fast = SpiHost(dut, half=4)
    for b in stream[8:]:
        await fast.write(SM(0) + TXF, b)

    await ClockCycles(dut.clk, 5500)

    data, wire_bits, bit_ns = usb.decode()
    crc = usb_crc16(payload)
    assert data == [pid] + payload + [crc & 0xFF, (crc >> 8) & 0xFF], \
        f"USB side broke under concurrency: {data}"
    assert not led.bad_pulses, f"WS2812 pulses out of spec: {led.bad_pulses}"
    assert led.frames and led.frames[0] == grb, \
        f"WS2812 side broke under concurrency: {led.frames}"
    dut._log.info(
        f"concurrent: USB {len(data)} bytes @ {bit_ns:.1f} ns/bit + "
        f"WS2812 {len(led.frames[0])} bytes, both clean")


@cocotb.test()
async def test_ps2_device(dut):
    """SM0 is a PS/2 device transmitter: DATA on GPIO0, device-generated
    CLK on GPIO1 via side-set, 12.5 kHz (divider 250, 16 ticks per bit).
    Each 11-bit frame (start, 8 data LSB first, odd parity, stop) spans
    two FIFO bytes with autopull refilling mid-frame. A host model
    samples DATA on falling CLK edges and validates framing, parity and
    the 10-16.7 kHz clock band.
    """
    spi = await setup(dut)
    host = Ps2Host(dut)
    cocotb.start_soon(host.run())

    # the program and settings of sw/examples/ps2_device.py: a 5 us
    # tick, 80 us a bit
    await spi.load_program(0, ps2_device.PROG)
    await configure(spi, ps2_device.config(clkdiv=250))
    await spi.write(R_CTRL, 0x11)

    scancodes = [0x1C, 0xF0]  # 'A' make, break prefix
    for code in scancodes:
        await spi.write(SM(0) + TXF, ps2_device.frame_bytes(code))

    # 2 frames x 11 bits x 80 us = 1.76 ms
    await ClockCycles(dut.clk, 100000)

    assert not host.errors, f"PS/2 framing errors: {host.errors}"
    assert host.bytes == scancodes, f"PS/2 {host.bytes} != {scancodes}"
    period = sum(host.periods_ns[:10]) / 10
    freq = 1e9 / period
    dut._log.info(f"PS/2 clock {freq/1e3:.2f} kHz")
    assert 10e3 <= freq <= 16.7e3, f"PS/2 clock {freq} Hz out of band"


@cocotb.test()
async def test_capture_replay(dut):
    """Waveform generator + logic analyzer: SM1 replays an arbitrary
    4-bit waveform on GPIO4-7 from its TX FIFO (OUT PINS,4) while SM0
    samples the same pins (IN PINS,4, autopush) at the same rate into
    its RX FIFO. Capture freezes itself when the RX FIFO fills - a
    16-sample snapshot - and the host reads the waveform back. One chip,
    both ends of a logic analyzer.
    """
    spi = await setup(dut)
    cocotb.start_soon(uio_loopback(dut))

    # the programs and settings of sw/examples/capture_replay.py: SM0
    # samples and SM1 replays, 2 ticks a sample
    await spi.load_program(0, capture_replay.CAP_PROG)
    await spi.load_program(capture_replay.AWG0, capture_replay.AWG_PROG)
    await configure(spi, capture_replay.config(div=16))

    wave = [0x1, 0x3, 0x7, 0xF, 0xE, 0xC, 0x8, 0x0,
            0x5, 0xA, 0x5, 0xA, 0x9, 0x6, 0x3, 0xC]
    packed = [(wave[i] << 4) | wave[i + 1] for i in range(0, 16, 2)]

    # preload the whole waveform (exactly one FIFO), then start both SMs
    # in the same CTRL write so replay and capture run in lockstep
    await spi.write(R_CTRL, 0x30)
    await spi.write(SM(1) + TXF, packed)
    await spi.write(R_CTRL, 0x03)

    await ClockCycles(dut.clk, 1400)  # 16 samples x 32 clk + margin

    fstat = (await spi.read(R_FSTAT, 1))[0]
    assert fstat & 0x08, "capture FIFO never filled (snapshot not taken)"
    raw = []
    for _ in range(8):
        raw += await spi.read(SM(0) + RXF, 1)
    samples = []
    for b in raw:
        samples += [b >> 4, b & 0xF]

    # constant pipeline lag (pin router + 2FF sync) plus start alignment:
    # compare transition sequences instead of absolute sample indices
    def dedup(seq):
        out = [seq[0]]
        for s in seq[1:]:
            if s != out[-1]:
                out.append(s)
        return out

    got = dedup(samples)
    if got and got[0] == 0 and wave[0] != 0:
        got = got[1:]  # pins idle low until the first replayed symbol
    assert len(got) >= 12, f"too few transitions captured: {samples}"
    assert got == wave[:len(got)], f"capture {got} != waveform {wave[:len(got)]}"
    dut._log.info(f"capture/replay: {len(samples)} samples, "
                  f"{len(got)} symbols reconstructed exactly")


@cocotb.test()
async def test_nec_ir_receive(dut):
    """SM0 decodes NEC infrared (post-demodulator polarity) on GPIO8 by
    measuring gap length: each bit is WAIT for burst, WAIT for burst end,
    then one sample a fixed delay later - still inside the next burst for
    a '0', still idle for a '1'. Referencing the sample to the burst end
    makes the loop self-correcting for both bit lengths. The leader is
    consumed without sampling, so exactly 32 bits autopush as 4 bytes.

    NEC timing is scaled 20x (28.1 us unit instead of 562.5 us) purely to
    keep the simulation short; every ratio is the real protocol's.
    """
    spi = await setup(dut)

    # the program and settings of sw/examples/nec_ir_receive.py, with
    # the divider scaled like the timing: a 2 us tick
    await spi.load_program(0, nec_ir_receive.PROG)
    await configure(spi, nec_ir_receive.config(clkdiv=100))
    # idle the IR line high before enabling, or WAIT(0) fires on reset state
    dut.ui_in.value = int(dut.ui_in.value) | (1 << 3)
    await ClockCycles(dut.clk, 2)
    await spi.write(R_CTRL, 0x11)

    addr, cmd = 0x00, 0x2F
    frame = [addr, addr ^ 0xFF, cmd, cmd ^ 0xFF]
    await nec_transmit(dut, frame, unit_ns=28125)  # 562.5 us / 20

    for _ in range(200):
        if not (await spi.read(R_FSTAT, 1))[0] & 0x04:
            break
        await ClockCycles(dut.clk, 500)
    got = []
    for _ in range(4):
        got += await spi.read(SM(0) + RXF, 1)
    assert got == frame, f"NEC decode {got} != {frame}"
    assert got[0] ^ got[1] == 0xFF and got[2] ^ got[3] == 0xFF
    dut._log.info(f"NEC: address {got[0]:#04x} command {got[2]:#04x} OK")


@cocotb.test()
async def test_spi_fuzz(dut):
    """Host-interface fuzz: random read/write bursts against instruction
    memory (side-effect-free RAM) interleaved with transfers aborted by
    CS_N mid-byte at random bit offsets. Semantics under test: only
    fully clocked bytes commit, partial bytes vanish, the address
    auto-increments per completed data byte, and an abort never corrupts
    state for the next transaction.
    """
    import random as _random
    spi = await setup(dut)
    rng = _random.Random(2027)

    async def clock_bits(byte_list, nbits):
        """Clock exactly nbits of byte_list under one CS, then abort."""
        bits = []
        for b in byte_list:
            bits += [(b >> i) & 1 for i in range(7, -1, -1)]
        spi._drive(0, 0, 0)
        await ClockCycles(dut.clk, SPI_HALF)
        for bit in bits[:nbits]:
            spi._drive(0, 0, bit)
            await ClockCycles(dut.clk, SPI_HALF)
            spi._drive(1, 0, bit)
            await ClockCycles(dut.clk, SPI_HALF)
        spi._drive(0, 0, 0)
        await ClockCycles(dut.clk, SPI_HALF)
        spi._drive(0, 1, 0)
        await ClockCycles(dut.clk, SPI_HALF)

    model = {a: 0 for a in range(0x40)}  # imem bytes, reset to X-free zero
    await spi.write(0x00, [0] * 0x40)    # make hardware match

    for step in range(60):
        kind = rng.random()
        addr = rng.randrange(0x40)
        if kind < 0.35:  # clean write burst
            n = rng.randrange(1, 5)
            data = [rng.randrange(256) for _ in range(n)]
            await spi.write(addr, data)
            for i, b in enumerate(data):
                if addr + i < 0x40:
                    model[addr + i] = b
        elif kind < 0.60:  # clean read burst, checked
            n = rng.randrange(1, 5)
            got = await spi.read(addr, n)
            exp = [model.get(addr + i, 0) for i in range(n)]
            exp = [e if addr + i < 0x40 else e
                   for i, e in enumerate(exp)]
            if addr + n <= 0x40:
                assert got == exp, f"step {step}: read {got} != {exp}"
        else:  # aborted transfer at a random bit offset
            n = rng.randrange(1, 4)
            data = [rng.randrange(256) for _ in range(n)]
            total_bits = 8 * (1 + n)
            nbits = rng.randrange(1, total_bits)  # always cut short
            write = rng.random() < 0.5
            cmd = (0x80 | addr) if write else addr
            await clock_bits([cmd] + data, nbits)
            if write:
                done = nbits // 8  # completed bytes incl. command
                for i in range(max(0, done - 1)):
                    if addr + i < 0x40:
                        model[addr + i] = data[i]
            # aborted reads have no architectural effect on imem

    got = await spi.read(0x00, 0x40)
    exp = [model[a] for a in range(0x40)]
    assert got == exp, (
        "post-fuzz imem mismatch at "
        f"{[hex(a) for a in range(0x40) if got[a] != exp[a]]}")
    dut._log.info("SPI fuzz: 60 transactions incl. mid-byte aborts, "
                  "imem state matches the reference model exactly")


@cocotb.test()
async def test_random_cosim_pins(dut):
    """Constrained-random cosim over the pin datapath: random programs
    with SET/OUT/MOV to pins and pindirs, IN/MOV from pins, JMP PIN,
    WAIT on a pin, computed jumps, MOV EXEC, delays and random side-set
    bits (optional and non-optional), random pin base configs across the
    whole 16-pin space. Runs at divider 8 so the pad
    loopback + 2FF sync settle between ticks; pin state carries across
    seeds like the real project-level registers do. Compares PC, RX FIFO
    contents and the GPIO_IN readback against the golden model.
    """
    spi = await setup(dut)
    cocotb.start_soon(uio_loopback(dut))

    # zero all pin/dir latches: earlier tests in this sim leave state
    zero = [
        SET(SDST_PINS, 0), SET(SDST_PINDIRS, 0),
    ]
    await spi.load_program(0, zero + [JMP(2)])
    for base, cnt in ((0, 7), (7, 7), (14, 2)):
        await spi.write(SM(0) + PIN_SET, (cnt << 4) | base)
        await spi.write(R_CTRL, 0x11)
        await ClockCycles(dut.clk, 200)
        await spi.write(R_CTRL, 0x00)

    pins = dirs = 0
    cells, resampled, total_steps = collections.Counter(), 0, 0
    for seed in range(PIN_SEEDS):
        prog, cfg, tx, g, steps, used_seed = random_case_pins(
            seed, pins, dirs)

        await spi.write(SM(0) + WRAP_BOTTOM, 0)
        await spi.write(SM(0) + WRAP_TOP, 31)
        await spi.write(R_CTRL, 0x10)
        await spi.load_program(0, prog)
        await spi.write(SM(0) + CLKDIV_INT_L, 8)
        await spi.write(SM(0) + CLKDIV_FRAC, 0)
        await spi.write(SM(0) + SHIFTCTRL, cfg["shiftctrl"])
        await spi.write(SM(0) + THRESH, cfg["thresh"])
        await spi.write(SM(0) + STATUS_CFG, cfg["status_cfg"])
        await spi.write(SM(0) + PIN_OUT,
                        (cfg["out_count"] << 4) | cfg["out_base"])
        await spi.write(SM(0) + PIN_SET,
                        (cfg["set_count"] << 4) | cfg["set_base"])
        await spi.write(SM(0) + PIN_IN, cfg["in_base"])
        await spi.write(SM(0) + PIN_SIDE,
                        (cfg["side_opt"] << 6) | (cfg["side_count"] << 4)
                        | cfg["side_base"])
        await spi.write(SM(0) + JMP_PIN, cfg["jmp_pin"])
        if tx:
            await spi.write(SM(0) + TXF, tx)
        await spi.write(R_CTRL, 0x01)

        await ClockCycles(dut.clk, (g.ticks + 60) * 8)

        await spi.write(R_CTRL, 0x00)
        pc = (await spi.read(R_PC0, 1))[0]
        gin = await spi.read(R_GPIO_IN_L, 2)
        gin = gin[0] | (gin[1] << 8)
        rx = []
        while len(rx) <= 8 and not (await spi.read(R_FSTAT, 1))[0] & 0x04:
            rx += await spi.read(SM(0) + RXF, 1)

        assert pc == g.pc, f"seed {used_seed}: PC {pc} != {g.pc}"
        assert rx == g.rx, f"seed {used_seed}: RX {rx} != {g.rx}"
        exp_gin = g._gpio_in()
        assert gin == exp_gin, (
            f"seed {used_seed}: GPIO_IN {gin:#06x} != {exp_gin:#06x} "
            f"(pins {g.pins:#06x} dirs {g.dirs:#06x})")
        pins, dirs = g.pins, g.dirs  # pad latches persist across seeds
        cells.update(g.cov)  # counted only once the hardware agrees
        resampled += used_seed != seed
        total_steps += steps
        dut._log.info(
            f"pin cosim seed {used_seed}: {steps} steps, pc={pc}, "
            f"rx={len(rx)}, gpio_in={gin:#06x} OK")
    save_coverage("pins", PIN_SEEDS, resampled, total_steps, cells)


@cocotb.test()
async def test_out_only_pads(dut):
    """GPIO13-15 map to the dedicated output pads uo[4:2]. Drive them
    with SET PINS and check the pads themselves (the pin cosim only
    checks the internal readback path), including hold-last-value.
    """
    spi = await setup(dut)
    prog = [
        SET(SDST_PINS, 0x5),   # 0: GPIO13=1, GPIO14=0, GPIO15=1
        JMP(1),                # 1: park
        SET(SDST_PINS, 0x2),   # 2: GPIO14=1, others low
        JMP(3),                # 3: park
    ]
    await spi.load_program(0, prog)
    await spi.write(SM(0) + CLKDIV_INT_L, 1)
    await spi.write(SM(0) + WRAP_TOP, 31)
    await spi.write(SM(0) + PIN_SET, 0x3D)  # base 13, count 3

    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(R_CTRL, 0x11)
    await ClockCycles(dut.clk, 20)
    pads = (int(dut.uo_out.value) >> 2) & 0x7
    assert pads == 0x5, f"uo[4:2] = {pads:#03b}, expected 0b101"

    # restart into the second program; pads must follow, then hold
    await spi.write(SM(0) + WRAP_BOTTOM, 2)
    await spi.write(R_CTRL, 0x11)
    await ClockCycles(dut.clk, 20)
    pads = (int(dut.uo_out.value) >> 2) & 0x7
    assert pads == 0x2, f"uo[4:2] = {pads:#03b}, expected 0b010"

    await spi.write(R_CTRL, 0x00)  # disabled: pads hold last value
    await ClockCycles(dut.clk, 20)
    pads = (int(dut.uo_out.value) >> 2) & 0x7
    assert pads == 0x2, f"pads did not hold after disable: {pads:#03b}"
    dut._log.info("out-only pads uo[4:2] drive and hold verified")


# ---------------------------------------------------------------------------
# SPI modes 0 to 3: the programs of sw/examples/spi_modes.py
# ---------------------------------------------------------------------------

async def spi_mode_case(dut, mode):
    """SM0 is the master and SM1 the slave of one SPI mode, full duplex.

    A monitor that knows only the mode decodes the three wires. It checks
    the idle level of SCK, which edge samples, and that MOSI and MISO hold
    still around every sampling edge.
    """
    spi = await setup(dut)
    cocotb.start_soon(uio_loopback(dut))
    mon = SpiMonitor(dut, mode)
    cocotb.start_soon(mon.run())

    await spi.load_program(spi_modes.MASTER_ORIGIN, spi_modes.master_program(mode))
    await spi.load_program(spi_modes.SLAVE_ORIGIN, spi_modes.slave_program(mode))
    await configure(spi, spi_modes.master_config() + spi_modes.slave_config(mode))

    await spi.write(R_CTRL, 0x33)       # both enabled and restarted
    await ClockCycles(dut.clk, 60)      # the master waits at its PULL
    mon.arm()                           # SCK has to rest at the idle level

    mosi_bytes = [0x9E, 0x01, 0x55, 0xC3]
    miso_bytes = [0xEF, 0x40, 0xAA, 0x81]
    for b in miso_bytes:
        await spi.write(SM(1) + TXF, b)
    for b in mosi_bytes:
        await spi.write(SM(0) + TXF, b)

    m_rx, s_rx = [], []
    for _ in range(300):
        fstat = (await spi.read(R_FSTAT, 1))[0]
        if not fstat & 0x04:  # SM0 RX non-empty
            m_rx += await spi.read(SM(0) + RXF, 1)
        if not fstat & 0x40:  # SM1 RX non-empty
            s_rx += await spi.read(SM(1) + RXF, 1)
        if len(m_rx) == 4 and len(s_rx) == 4:
            break
        await ClockCycles(dut.clk, 20)
    await ClockCycles(dut.clk, 60)

    errors = mon.finish()
    assert not errors, f"mode {mode}: {errors}"
    assert mon.leading == 32, f"mode {mode}: {mon.leading} clock pulses, not 32"
    assert mon.mosi_bytes == mosi_bytes, f"mode {mode}: MOSI on the wire {mon.mosi_bytes}"
    assert mon.miso_bytes == miso_bytes, f"mode {mode}: MISO on the wire {mon.miso_bytes}"
    assert s_rx == mosi_bytes, f"mode {mode}: slave RX {s_rx} != {mosi_bytes}"
    assert m_rx == miso_bytes, f"mode {mode}: master RX {m_rx} != {miso_bytes}"


@cocotb.test()
async def test_spi_mode0(dut):
    """SPI mode 0: SCK idles low, data sampled on the rising edge."""
    await spi_mode_case(dut, 0)


@cocotb.test()
async def test_spi_mode1(dut):
    """SPI mode 1: SCK idles low, data sampled on the falling edge."""
    await spi_mode_case(dut, 1)


@cocotb.test()
async def test_spi_mode2(dut):
    """SPI mode 2: SCK idles high, data sampled on the falling edge."""
    await spi_mode_case(dut, 2)


@cocotb.test()
async def test_spi_mode3(dut):
    """SPI mode 3: SCK idles high, data sampled on the rising edge."""
    await spi_mode_case(dut, 3)


# ---------------------------------------------------------------------------
# I2C master read: the program of sw/examples/i2c_read.py
# ---------------------------------------------------------------------------

async def i2c_read_start(dut, speed, **target):
    """Load the read program at one speed, beside a bus with one target."""
    spi = await setup(dut)
    bus = I2CTarget(dut, **target)
    cocotb.start_soon(bus.run())
    await spi.load_program(0, i2c_read.PROG)
    await configure(spi, i2c_read.config(speed))
    await spi.write(R_CTRL, 0x11)       # SM0 enabled and restarted
    return spi, bus


async def i2c_read_txn(spi, dut, addr7, n, polls=600):
    """The steps of i2c_read.read_txn, on the test bench's SPI host."""
    done = 1 << i2c_read.DONE_IRQ
    await spi.load_program(i2c_read.SET_Y_ADDR, [i2c_read.count_patch(n)])
    await spi.write(R_IRQ, done)
    await spi.write(SM(0) + TXF, i2c_read.address_byte(addr7))
    for _ in range(polls):
        if (await spi.read(R_IRQ, 1))[0] & done:
            break
        await ClockCycles(dut.clk, 50)
    else:
        raise AssertionError("the transaction did not raise its flag")
    await spi.write(R_IRQ, done)
    await ClockCycles(dut.clk, 1200)    # the STOP follows the flag
    level = (await spi.read(R_FLEVEL0, 1))[0] >> 4
    data = []
    for _ in range(level):
        data += await spi.read(SM(0) + RXF, 1)
    return data or None


def i2c_check_timing(bus, speed, needs):
    """Every bus time in the record meets the specification for `speed`."""
    got, spec = bus.timing(), I2C_SPEC_NS[speed]
    missing = [k for k in needs if k not in got]
    assert not missing, f"the record has no {missing}"
    short = {k: (got[k], spec[k]) for k in spec if k in got and got[k] < spec[k]}
    assert not short, f"{speed} mode: measured below the minimum, in ns: {short}"
    assert got["t_hd_dat"] > 0, "SDA changed on the falling edge of SCL"
    return got


@cocotb.test()
async def test_i2c_read_fast(dut):
    """Read three bytes at fast-mode speed, 400 kHz.

    The master acknowledges every byte but the last. The bus times are
    measured on the wires and held against the fast-mode minimums.
    """
    data = [0xA5, 0x3C, 0xF0]
    spi, bus = await i2c_read_start(dut, i2c_read.FAST, addr7=0x52, read_data=data)
    got = await i2c_read_txn(spi, dut, 0x52, 3)

    assert got == data, f"read {got}, and the target holds {data}"
    assert bus.addressed == [(0x52, 1)], f"address byte {bus.addressed}"
    assert bus.sent == data and bus.master_acks == [True, True, False], \
        f"sent {bus.sent}, acknowledged {bus.master_acks}"
    assert len(bus.starts) == 1 and len(bus.stops) == 1, \
        f"{len(bus.starts)} START and {len(bus.stops)} STOP conditions"
    rises = sum(1 for _, level in bus.scl_edges if level)
    assert rises == 36 + 1, f"{rises} rises of SCL, not 36 clocks and the STOP"
    t = i2c_check_timing(bus, "fast", ["period", "t_hd_sta", "t_low", "t_high",
                                       "t_su_dat", "t_su_sto"])
    dut._log.info("fast mode, ns: %s", t)
    assert (await spi.read(R_PC0, 1))[0] == 1, "the master is not back at its PULL"


@cocotb.test()
async def test_i2c_read_standard_stretch(dut):
    """Read at standard-mode speed, 100 kHz, from a target that stretches
    the clock.

    The target holds SCL low for about 12 us before each byte, more than
    twice the master's own low time. Three reads use three hold lengths, a
    third of a tick apart, so the release falls at different points of the
    master's tick. The master waits for the line each time, and the low
    and high times still meet the standard-mode minimums.

    The master sees its pins once a tick. So the clock period that begins
    with the end of a stretch can be up to one tick shorter than the rest:
    the test records it, and holds it to the sum of the two minimums.
    """
    data = [0x81, 0x7E, 0x3C, 0xD2]
    holds = (600, 611, 622)                         # clock cycles: 12 us and up
    spi, bus = await i2c_read_start(dut, i2c_read.STANDARD, addr7=0x52,
                                    read_data=data, stretch=holds[0])
    got = await i2c_read_txn(spi, dut, 0x52, 2)
    for hold in holds[1:]:
        bus.stretch = hold
        got += await i2c_read_txn(spi, dut, 0x52, 1)

    assert got == data, f"read {got}, and the target holds {data}"
    assert bus.master_acks == [True, False, False, False], f"acknowledged {bus.master_acks}"
    assert len(bus.starts) == 3 and len(bus.stops) == 3
    t = i2c_check_timing(bus, "standard", ["period", "t_hd_sta", "t_low", "t_high",
                                           "t_su_dat", "t_su_sto", "t_buf"])
    dut._log.info("standard mode, ns: %s", t)
    falls = [x for x, level in bus.scl_edges if not level]
    rises = [x for x, level in bus.scl_edges if level]
    lows = [min(r for r in rises if r > f) - f for f in falls if any(r > f for r in rises)]
    stretched = [low for low in lows if low >= holds[0] * 20]
    assert len(stretched) == 4, f"{len(stretched)} stretched low times, not 4: {sorted(lows)[-5:]}"
    # The high time that follows a stretch is the one the master has to
    # count from the line, not from its own release.
    after = []
    for f in falls:
        up = [r for r in rises if r > f]
        if up and up[0] - f >= holds[0] * 20:
            down = [x for x in falls if x > up[0]]
            after.append(down[0] - up[0])
    dut._log.info("high times after a stretch, ns: %s", after)
    spec = I2C_SPEC_NS["standard"]
    assert min(after) >= spec["t_high"], f"high times after a stretch {after}"
    assert t["period_after_stretch"] >= spec["t_low"] + spec["t_high"], \
        f"a period of {t['period_after_stretch']} ns after a stretch"


@cocotb.test()
async def test_i2c_read_no_device(dut):
    """An address that nobody acknowledges gets a STOP and no data clock.

    The next read, of the device that is there, then works, and the bus
    free time between the two meets the fast-mode minimum.
    """
    spi, bus = await i2c_read_start(dut, i2c_read.FAST, addr7=0x52, read_data=[0x6B])
    got = await i2c_read_txn(spi, dut, 0x29, 2)

    assert got is None, f"read {got} from an address with no device"
    assert bus.addressed == [(0x29, 1)], f"address byte {bus.addressed}"
    rises = sum(1 for _, level in bus.scl_edges if level)
    assert rises == 9 + 1, f"{rises} rises of SCL, not 9 clocks and the STOP"
    assert len(bus.starts) == 1 and len(bus.stops) == 1

    got = await i2c_read_txn(spi, dut, 0x52, 1)
    assert got == [0x6B], f"read {got} after the failed address"
    assert bus.addressed == [(0x29, 1), (0x52, 1)]
    assert bus.master_acks == [False]
    assert len(bus.starts) == 2 and len(bus.stops) == 2
    t = i2c_check_timing(bus, "fast", ["t_buf", "t_su_sto", "t_hd_sta"])
    dut._log.info("fast mode, ns: %s", t)
