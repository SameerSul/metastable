# Metastable protocol emulator - test helpers: SPI host driver + bus models
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# The ISA assembler and register map live in sw/metastable_asm.py (shared
# with the MicroPython host software) and are re-exported here so the tests
# exercise exactly what ships to the demo board.

import os
import sys

from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sw"))
from metastable_asm import *  # noqa: F401,F403


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


class WS2812Decoder:
    """Pulse-width decoder for WS2812/NeoPixel data on GPIO0 (uio[0]).

    Measures every high pulse against the WS2812B datasheet windows
    (T0H 250-550 ns, T1H 650-950 ns), shifts bits MSB first, and cuts a
    frame after a reset-length low gap. Records out-of-spec pulses.
    """

    RESET_NS = 9000  # datasheet says >50 us; shortened to keep sims fast

    def __init__(self, dut):
        self.dut = dut
        self.frames = []   # list of byte lists, one per latched frame
        self.bad_pulses = []
        self._bits = []

    def _cut(self):
        if self._bits:
            n = len(self._bits) - len(self._bits) % 8
            frame = [
                sum(b << (7 - i) for i, b in enumerate(self._bits[k:k + 8]))
                for k in range(0, n, 8)
            ]
            self.frames.append(frame)
            self._bits = []

    async def run(self):
        from cocotb.utils import get_sim_time
        dut = self.dut
        prev, t_rise, t_fall = 0, None, None
        while True:
            await RisingEdge(dut.clk)
            try:
                driven = int(dut.uio_oe.value) & 1
                level = int(dut.uio_out.value) & driven
            except ValueError:
                driven, level = 0, 0
            now = get_sim_time("ns")
            if level and not prev:
                t_rise = now
                t_fall = None
            elif prev and not level:
                t_fall = now
                width = now - t_rise
                if 250 <= width <= 550:
                    self._bits.append(0)
                elif 650 <= width <= 950:
                    self._bits.append(1)
                else:
                    self.bad_pulses.append(width)
            elif not level and t_fall is not None and now - t_fall > self.RESET_NS:
                self._cut()
                t_fall = None
            prev = level
