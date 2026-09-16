# Metastable protocol emulator - test helpers: ISA assembler + SPI host driver
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0

from cocotb.triggers import ClockCycles, RisingEdge

# ---------------------------------------------------------------------------
# ISA assembler
# ---------------------------------------------------------------------------

# IN/MOV sources
SRC_PINS, SRC_X, SRC_Y, SRC_NULL, SRC_STATUS, SRC_ISR, SRC_OSR = 0, 1, 2, 3, 5, 6, 7
# OUT destinations
ODST_PINS, ODST_X, ODST_Y, ODST_NULL, ODST_PINDIRS, ODST_PC, ODST_ISR = 0, 1, 2, 3, 4, 5, 6
# MOV destinations
MDST_PINS, MDST_X, MDST_Y, MDST_EXEC, MDST_PC, MDST_ISR, MDST_OSR = 0, 1, 2, 4, 5, 6, 7
# MOV operations
M_COPY, M_INV, M_REV = 0, 1, 2
# SET destinations
SDST_PINS, SDST_X, SDST_Y, SDST_PINDIRS = 0, 1, 2, 4
# JMP conditions
C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC, C_XNEY, C_PIN, C_NOTOSRE = range(8)
# WAIT sources
W_GPIO, W_PIN, W_IRQ = 0, 1, 2


def _dss(delay, side):
    # side must be pre-positioned: value << (5 - side_count),
    # e.g. side=0x10 drives 1 on the side-set pin when side_count == 1.
    # Use side(val, count, opt) to build this.
    assert 0 <= delay <= 31
    return ((delay | side) & 0x1F) << 8


def side(val, count, opt=False):
    """Position a side-set value for the delay/side field.

    With opt=True (PIN_SIDE bit 6 set), bit 4 is the per-instruction
    enable and the side bits sit one position lower; instructions
    without side() then leave the side-set pins untouched.
    """
    assert 1 <= count <= 3 and 0 <= val < (1 << count)
    if opt:
        return 0x10 | (val << (4 - count))
    return val << (5 - count)


def JMP(addr, cond=C_ALWAYS, delay=0, side=0):
    return (0 << 13) | _dss(delay, side) | (cond << 5) | (addr & 0x1F)

def WAIT(pol, src, idx, delay=0, side=0):
    return (1 << 13) | _dss(delay, side) | (pol << 7) | (src << 5) | (idx & 0x1F)

def IN_(src, n, delay=0, side=0):
    return (2 << 13) | _dss(delay, side) | (src << 5) | (n & 0x1F)

def OUT(dst, n, delay=0, side=0):
    return (3 << 13) | _dss(delay, side) | (dst << 5) | (n & 0x1F)

def PUSH(iffull=0, block=1, delay=0, side=0):
    return (4 << 13) | _dss(delay, side) | (0 << 7) | (iffull << 6) | (block << 5)

def PULL(ifempty=0, block=1, delay=0, side=0):
    return (4 << 13) | _dss(delay, side) | (1 << 7) | (ifempty << 6) | (block << 5)

def MOV(dst, src, op=0, delay=0, side=0):
    return (5 << 13) | _dss(delay, side) | (dst << 5) | (op << 3) | src

def IRQ(idx, clr=0, wait=0, delay=0, side=0):
    return (6 << 13) | _dss(delay, side) | (clr << 6) | (wait << 5) | (idx & 0x3)

def SET(dst, imm, delay=0, side=0):
    return (7 << 13) | _dss(delay, side) | (dst << 5) | (imm & 0x1F)


# ---------------------------------------------------------------------------
# register map
# ---------------------------------------------------------------------------

R_CTRL, R_FSTAT, R_IRQ, R_IRQ_MASK = 0x40, 0x41, 0x42, 0x43
R_GPIO_IN_L, R_GPIO_IN_H, R_PC0, R_PC1 = 0x44, 0x45, 0x46, 0x47

def SM(n):  # per-SM register base
    return 0x50 + 0x10 * n

CLKDIV_INT_L, CLKDIV_INT_H, CLKDIV_FRAC = 0x0, 0x1, 0x2
WRAP_TOP, WRAP_BOTTOM, SHIFTCTRL, THRESH = 0x3, 0x4, 0x5, 0x6
PIN_OUT, PIN_SET, PIN_IN, PIN_SIDE, JMP_PIN = 0x7, 0x8, 0x9, 0xA, 0xB
TXF, RXF = 0xC, 0xD

# SHIFTCTRL bits
AUTOPULL, AUTOPUSH, OUT_RIGHT, IN_RIGHT = 1, 2, 4, 8

# PIN_SIDE bits: [3:0] base, [5:4] count, [6] optional, [7] drive pindirs
SIDE_OPT, SIDE_PINDIR = 0x40, 0x80


# ---------------------------------------------------------------------------
# SPI host driver (mode 0). ui_in: [0] SCK, [1] CS_N, [2] MOSI. uo_out[0] MISO.
# ---------------------------------------------------------------------------

SPI_HALF = 10  # clk cycles per half SCK period


class SpiHost:
    def __init__(self, dut):
        self.dut = dut
        self._ui = 0x02  # CS_N high, SCK low

    def _drive(self, sck, cs_n, mosi):
        base = int(self.dut.ui_in.value) & 0xF8
        self.dut.ui_in.value = base | (sck) | (cs_n << 1) | (mosi << 2)

    async def _xfer(self, tx_bytes):
        dut = self.dut
        rx = []
        self._drive(0, 0, 0)
        await ClockCycles(dut.clk, SPI_HALF)
        for b in tx_bytes:
            r = 0
            for i in range(7, -1, -1):
                self._drive(0, 0, (b >> i) & 1)
                await ClockCycles(dut.clk, SPI_HALF)
                # sample MISO only (other uo bits may be X, e.g. unwritten imem)
                miso = dut.uo_out.value.binstr[-1]
                r = (r << 1) | (1 if miso == "1" else 0)
                self._drive(1, 0, (b >> i) & 1)
                await ClockCycles(dut.clk, SPI_HALF)
            rx.append(r)
        self._drive(0, 0, 0)
        await ClockCycles(dut.clk, SPI_HALF)
        self._drive(0, 1, 0)
        await ClockCycles(dut.clk, SPI_HALF)
        return rx

    async def write(self, addr, data):
        if isinstance(data, int):
            data = [data]
        await self._xfer([0x80 | addr] + list(data))

    async def read(self, addr, n=1):
        rx = await self._xfer([addr & 0x7F] + [0] * n)
        return rx[1:]

    async def load_program(self, origin, instrs):
        data = []
        for w in instrs:
            data += [w & 0xFF, (w >> 8) & 0xFF]
        await self.write(2 * origin, data)


async def trace_pin0(dut):
    """Log every transition of GPIO0 (uio_out[0]) with timestamps."""
    import cocotb.utils
    prev = None
    while True:
        await RisingEdge(dut.clk)
        bs = dut.uio_out.value.binstr
        oe = dut.uio_oe.value.binstr
        cur = (bs[-1], oe[-1])
        if cur != prev:
            t = cocotb.utils.get_sim_time("ns")
            dut._log.info(f"GPIO0 out={cur[0]} oe={cur[1]} @ {t}ns")
            prev = cur


class I2CSlave:
    """Open-drain I2C bus model + slave. SDA = GPIO0, SCL = GPIO1.

    Resolves the wired-AND bus from the DUT's uio_out/uio_oe and its own
    ACK driver (released lines pull high), reflects the result onto
    uio_in, samples data on SCL rise, ACKs every byte, and records
    START/STOP conditions. Other uio bits are looped back as usual.
    """

    def __init__(self, dut):
        self.dut = dut
        self.bytes = []
        self.started = False
        self.stopped = False
        self._ack = False
        self._bit = 0
        self._sh = 0

    async def run(self):
        dut = self.dut
        prev_sda, prev_scl = 1, 1
        while True:
            await RisingEdge(dut.clk)
            try:
                out = int(dut.uio_out.value)
                oe = int(dut.uio_oe.value)
            except ValueError:
                out, oe = 0, 0
            sda = 0 if ((oe & ~out & 1) or self._ack) else 1
            scl = 0 if (oe & ~out & 2) else 1
            if scl and prev_scl:  # SDA edge while SCL high
                if prev_sda and not sda:
                    self.started = True
                    self._bit, self._sh = 0, 0
                elif sda and not prev_sda:
                    self.started = False
                    self.stopped = True
            if self.started:
                if scl and not prev_scl:  # sample on SCL rise
                    self._bit += 1
                    if self._bit <= 8:
                        self._sh = ((self._sh << 1) | sda) & 0xFF
                elif prev_scl and not scl:  # drive on SCL fall
                    if self._bit == 8:
                        self.bytes.append(self._sh)
                        self._ack = True
                    elif self._bit == 9:
                        self._ack = False
                        self._bit, self._sh = 0, 0
            dut.uio_in.value = (out & oe & 0xFC) | (scl << 1) | sda
            prev_sda, prev_scl = sda, scl


async def uio_loopback(dut):
    """Reflect driven uio outputs back onto uio_in, as the pads would."""
    while True:
        await RisingEdge(dut.clk)
        try:
            out = int(dut.uio_out.value)
            oe = int(dut.uio_oe.value)
        except ValueError:
            out, oe = 0, 0
        dut.uio_in.value = out & oe
