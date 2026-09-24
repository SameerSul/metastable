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
    def __init__(self, dut, half=SPI_HALF):
        self.dut = dut
        self.half = half  # clk cycles per half SCK period (>= 4: clk >= 8x SCK)
        self._ui = 0x02  # CS_N high, SCK low

    def _drive(self, sck, cs_n, mosi):
        base = int(self.dut.ui_in.value) & 0xF8
        self.dut.ui_in.value = base | (sck) | (cs_n << 1) | (mosi << 2)

    async def _xfer(self, tx_bytes):
        dut = self.dut
        rx = []
        self._drive(0, 0, 0)
        await ClockCycles(dut.clk, self.half)
        for b in tx_bytes:
            r = 0
            for i in range(7, -1, -1):
                self._drive(0, 0, (b >> i) & 1)
                await ClockCycles(dut.clk, self.half)
                # sample MISO only (other uo bits may be X, e.g. unwritten imem)
                miso = dut.uo_out.value.binstr[-1]
                r = (r << 1) | (1 if miso == "1" else 0)
                self._drive(1, 0, (b >> i) & 1)
                await ClockCycles(dut.clk, self.half)
            rx.append(r)
        self._drive(0, 0, 0)
        await ClockCycles(dut.clk, self.half)
        self._drive(0, 1, 0)
        await ClockCycles(dut.clk, self.half)
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

    def __init__(self, dut, pin=0):
        self.dut = dut
        self.pin = pin
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
                driven = (int(dut.uio_oe.value) >> self.pin) & 1
                level = (int(dut.uio_out.value) >> self.pin) & driven
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


# ---------------------------------------------------------------------------
# USB low-speed helpers: host-side symbol encoder + bus decoder model
# ---------------------------------------------------------------------------
# Low-speed signalling: J = D- high / D+ low, K = D+ high / D- low,
# SE0 = both low. With D+ on GPIO0 and D- on GPIO1, OUT PINS,2 takes the
# symbol as {D-, D+}: J = 0b10, K = 0b01, SE0 = 0b00.

USB_J, USB_K, USB_SE0 = 0b10, 0b01, 0b00


def usb_crc16(data):
    """CRC16-USB (poly 0x8005 reflected, init 0xFFFF), complemented."""
    crc = 0xFFFF
    for b in data:
        for i in range(8):
            if (crc ^ (b >> i)) & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return (~crc) & 0xFFFF


def usb_ls_packet_symbols(pid, payload):
    """Build one LS packet as a J/K/SE0 symbol list: sync + PID (+ data +
    CRC16) with NRZI encoding ('0' toggles, '1' holds) and bit stuffing
    (a 0 forced after six 1s), then the SE0/SE0/J EOP."""
    packet = [pid] + list(payload)
    if payload:
        crc = usb_crc16(payload)
        packet += [crc & 0xFF, (crc >> 8) & 0xFF]
    bits = [0, 0, 0, 0, 0, 0, 0, 1]  # sync byte 0x80, LSB first
    for b in packet:
        bits += [(b >> i) & 1 for i in range(8)]
    stuffed, ones = [], 0
    for bit in bits:
        stuffed.append(bit)
        ones = ones + 1 if bit else 0
        if ones == 6:
            stuffed.append(0)
            ones = 0
    symbols, state = [], USB_J  # bus idles at J
    for bit in stuffed:
        if not bit:
            state = USB_K if state == USB_J else USB_J
        symbols.append(state)
    return symbols + [USB_SE0, USB_SE0, USB_J]


def usb_symbols_to_bytes(symbols, lead_idle=8):
    """Pack symbols 4 per FIFO byte for OUT PINS,2 shifting right, with a
    leading idle-J run and the final byte padded with idle J."""
    syms = [USB_J] * lead_idle + list(symbols)
    while len(syms) % 4:
        syms.append(USB_J)
    return [
        syms[i] | (syms[i + 1] << 2) | (syms[i + 2] << 4) | (syms[i + 3] << 6)
        for i in range(0, len(syms), 4)
    ]


class UsbLsDecoder:
    """Samples D+/D- (GPIO0/1) transitions and decodes one LS packet:
    locks onto the first J->K edge, samples at bit centers, NRZI-decodes,
    destuffs, and splits sync/packet/EOP. Also measures the bit rate."""

    T_BIT_NS = 666.875  # 8 ticks x (4 + 43/256) x 20 ns

    def __init__(self, dut):
        self.dut = dut
        self.edges = []  # (time_ns, {D-,D+} symbol)

    async def run(self):
        from cocotb.utils import get_sim_time
        dut = self.dut
        prev = None
        while True:
            await RisingEdge(dut.clk)
            try:
                out = int(dut.uio_out.value)
                oe = int(dut.uio_oe.value)
            except ValueError:
                out, oe = 0, 0
            sym = out & oe & 3
            if sym != prev:
                self.edges.append((get_sim_time("ns"), sym))
                prev = sym

    def decode(self):
        """Returns (packet_bytes, n_wire_bits, measured_bit_ns) or raises."""
        start = next(t for t, s in self.edges if s == USB_K)  # sync KJKJ...
        def sample(n):
            t = start + (n + 0.5) * self.T_BIT_NS
            sym = USB_J
            for et, es in self.edges:
                if et <= t:
                    sym = es
                else:
                    break
            return sym
        symbols, n = [], 0
        while True:
            s = sample(n)
            if s == USB_SE0:
                break
            symbols.append(s)
            n += 1
            assert n < 4000, "no EOP found"
        assert sample(n + 1) == USB_SE0 and sample(n + 2) == USB_J, "bad EOP"
        eop_edge = next(t for t, s in self.edges if t > start and s == USB_SE0)
        measured = (eop_edge - start) / len(symbols)
        bits, state, ones = [], USB_J, 0
        for s in symbols:
            bit = 1 if s == state else 0
            state = s
            if ones == 6:  # stuffed bit: must be a 0, drop it
                assert bit == 0, "missing stuff bit"
                ones = 0
                continue
            ones = ones + 1 if bit else 0
            bits.append(bit)
        assert bits[:8] == [0, 0, 0, 0, 0, 0, 0, 1], f"bad sync {bits[:8]}"
        bits = bits[8:]
        data = [
            sum(bits[k + i] << i for i in range(8))
            for k in range(0, len(bits) - len(bits) % 8, 8)
        ]
        return data, len(symbols), measured


class Ps2Host:
    """PS/2 host model: DATA = GPIO0, CLK = GPIO1 (device-driven).

    Samples DATA on every falling CLK edge, validates the 11-bit frame
    (start 0, 8 data bits LSB first, odd parity, stop 1), and records
    the clock period. Undriven lines read high (pull-ups).
    """

    def __init__(self, dut):
        self.dut = dut
        self.bytes = []
        self.errors = []
        self.periods_ns = []
        self._bits = []
        self._t_fall = None

    async def run(self):
        from cocotb.utils import get_sim_time
        dut = self.dut
        prev_clk = 1
        while True:
            await RisingEdge(dut.clk)
            try:
                out = int(dut.uio_out.value)
                oe = int(dut.uio_oe.value)
            except ValueError:
                out, oe = 0, 0
            data = (out >> 0) & 1 if (oe >> 0) & 1 else 1
            clkl = (out >> 1) & 1 if (oe >> 1) & 1 else 1
            if prev_clk and not clkl:  # falling edge: sample
                now = get_sim_time("ns")
                if self._t_fall is not None:
                    self.periods_ns.append(now - self._t_fall)
                self._t_fall = now
                self._bits.append(data)
                if len(self._bits) == 11:
                    b = self._bits
                    val = sum(bit << i for i, bit in enumerate(b[1:9]))
                    if b[0] != 0:
                        self.errors.append("start bit high")
                    elif (sum(b[1:10]) % 2) != 1:
                        self.errors.append(f"parity error on {val:#x}")
                    elif b[10] != 1:
                        self.errors.append("stop bit low")
                    else:
                        self.bytes.append(val)
                    self._bits = []
            prev_clk = clkl


def ps2_frame_bytes(value):
    """Pack one PS/2 device frame (start, 8 data LSB first, odd parity,
    stop) into two FIFO bytes for OUT PINS,1 shifting right."""
    parity = 1 ^ (bin(value).count("1") & 1)
    bits = [0] + [(value >> i) & 1 for i in range(8)] + [parity, 1]
    bits += [1] * 5  # pad the second byte with idle-high bits
    return [sum(bits[k + i] << i for i in range(8)) for k in (0, 8)]
