# Metastable protocol emulator - cocotb tests
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from metastable import (
    SpiHost, uio_loopback,
    JMP, WAIT, IN_, OUT, PUSH, PULL, MOV, IRQ, SET,
    SRC_PINS, ODST_PINS, SDST_PINS, SDST_PINDIRS, SDST_X,
    C_XDEC, W_PIN,
    R_CTRL, R_FSTAT, R_IRQ, R_GPIO_IN_H, SM,
    CLKDIV_INT_L, WRAP_TOP, WRAP_BOTTOM, PIN_OUT, PIN_SET, PIN_IN, PIN_SIDE,
    TXF, RXF,
)


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

    # UART TX, 8N1, LSB first, 8 ticks/bit, TX = GPIO0 (SET + OUT pins)
    tx_prog = [
        SET(SDST_PINDIRS, 1),             # 0: GPIO0 output
        SET(SDST_PINS, 1),                # 1: line idle high
        PULL(block=1),                    # 2: stall (idle) until data
        SET(SDST_X, 7),                   # 3: bit counter
        SET(SDST_PINS, 0, delay=7),       # 4: start bit, 8 ticks
        OUT(ODST_PINS, 1),                # 5: shift one data bit
        JMP(5, C_XDEC, delay=6),          # 6: 8 ticks per bit
        SET(SDST_PINS, 1, delay=6),       # 7: stop bit, then wrap
    ]
    # UART RX on GPIO0, samples mid-bit
    RX0 = 8
    rx_prog = [
        WAIT(0, W_PIN, 0),           # 8: start bit edge
        SET(SDST_X, 7, delay=10),    # 9: align to centre of data bit 0
        IN_(SRC_PINS, 1),            # 10:
        JMP(10, C_XDEC, delay=6),    # 11: 8 ticks per bit
        WAIT(1, W_PIN, 0),           # 12: wait for stop bit (no false start)
        PUSH(block=0),               # 13:
    ]

    await spi.load_program(0, tx_prog)
    await spi.load_program(RX0, rx_prog)

    # SM0: divider 2, wrap 0..7, out 1 pin @0, set 1 pin @0
    await spi.write(SM(0) + CLKDIV_INT_L, 2)
    await spi.write(SM(0) + WRAP_TOP, 7)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(SM(0) + PIN_OUT, 0x10)   # base 0, count 1
    await spi.write(SM(0) + PIN_SET, 0x10)   # base 0, count 1

    # SM1: divider 2, wrap 8..13, in base 0
    await spi.write(SM(1) + CLKDIV_INT_L, 2)
    await spi.write(SM(1) + WRAP_TOP, 13)
    await spi.write(SM(1) + WRAP_BOTTOM, RX0)
    await spi.write(SM(1) + PIN_IN, 0)

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
