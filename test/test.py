# Metastable protocol emulator - cocotb tests
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from metastable import (
    SpiHost, uio_loopback, I2CSlave, side,
    JMP, WAIT, IN_, OUT, PUSH, PULL, MOV, IRQ, SET,
    SRC_PINS, SRC_X, SRC_Y, SRC_NULL, SRC_STATUS, SRC_OSR,
    ODST_PINS, ODST_NULL, ODST_PINDIRS, ODST_PC, ODST_ISR,
    MDST_X, MDST_Y, MDST_EXEC, MDST_ISR, M_INV, M_REV,
    SDST_PINS, SDST_PINDIRS, SDST_X, SDST_Y,
    C_XDEC, C_YDEC, C_XNEY, C_PIN, C_NOTOSRE, W_PIN, W_GPIO, W_IRQ,
    R_CTRL, R_FSTAT, R_IRQ, R_IRQ_MASK, R_GPIO_IN_H, R_PC0, R_FLEVEL0, SM,
    CLKDIV_INT_L, CLKDIV_FRAC, WRAP_TOP, WRAP_BOTTOM, SHIFTCTRL, THRESH,
    PIN_OUT, PIN_SET, PIN_IN, PIN_SIDE, JMP_PIN,
    AUTOPULL, AUTOPUSH, SIDE_OPT, SIDE_PINDIR,
    TXF, RXF, STATUS_CFG, STATUS_RX,
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

    OPT = True
    LO = side(1, 1, OPT)  # SCL dir=1: drive low
    HI = side(0, 1, OPT)  # SCL dir=0: release high
    prog = [
        SET(SDST_PINDIRS, 0),                    # 0: SDA released (bus idle)
        PULL(block=1),                           # 1: wait for first byte
        SET(SDST_PINDIRS, 1, delay=7),           # 2: START: SDA low, SCL high
        SET(SDST_Y, 1),                          # 3: byte count - 1
        SET(SDST_X, 7, side=LO, delay=1),        # 4: SCL low, bit counter
        OUT(ODST_PINDIRS, 1, side=LO, delay=3),  # 5: SDA <= bit (autopull)
        JMP(5, C_XDEC, side=HI, delay=3),        # 6: SCL high, slave samples
        SET(SDST_PINDIRS, 0, side=LO, delay=3),  # 7: release SDA for ACK
        IN_(SRC_PINS, 1, side=HI, delay=3),      # 8: sample ACK on SCL rise
        JMP(4, C_YDEC, side=LO, delay=1),        # 9: next byte
        SET(SDST_PINDIRS, 1, side=LO, delay=3),  # 10: SDA low ahead of STOP
        MOV(MDST_Y, SRC_Y, side=HI, delay=3),    # 11: SCL high, SDA held low
        SET(SDST_PINDIRS, 0, delay=7),           # 12: STOP: SDA rises
        PUSH(block=0),                           # 13: ACK bits -> host
    ]
    await spi.load_program(0, prog)

    await spi.write(SM(0) + CLKDIV_INT_L, 8)
    await spi.write(SM(0) + WRAP_TOP, 13)
    await spi.write(SM(0) + WRAP_BOTTOM, 0)
    await spi.write(SM(0) + SHIFTCTRL, AUTOPULL)  # shift left, MSB first
    await spi.write(SM(0) + PIN_OUT, 0x10)        # SDA: base 0, count 1
    await spi.write(SM(0) + PIN_SET, 0x10)        # SDA: base 0, count 1
    await spi.write(SM(0) + PIN_IN, 0)            # SDA
    # SCL: base 1, count 1, optional, drives pindirs
    await spi.write(SM(0) + PIN_SIDE, SIDE_PINDIR | SIDE_OPT | 0x11)

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
