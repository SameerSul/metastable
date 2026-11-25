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
# The example programs: the tests run the files that ship to the board.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sw", "examples"))
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

# The host side of the USB packet: built by the example that ships.
from usb_ls_tx import (  # noqa: E402
    J as USB_J, K as USB_K, SE0 as USB_SE0,
    crc16 as usb_crc16,
    packet_symbols as usb_ls_packet_symbols,
    symbols_to_bytes as usb_symbols_to_bytes,
)


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


# The host side of a PS/2 frame: packed by the example that ships.
from ps2_device import frame_bytes as ps2_frame_bytes  # noqa: E402


async def nec_transmit(dut, data_bytes, unit_ns, gpio_bit=3):
    """Drive an NEC IR frame (demodulator polarity: idle high, burst low)
    onto ui_in[gpio_bit]. unit_ns is the 562.5 us burst unit, scalable to
    keep simulations short. Leader 16/8 units, bit = 1 unit burst + 1
    ('0') or 3 ('1') units gap, then the stop burst."""
    from cocotb.triggers import Timer

    def drive(level):
        v = int(dut.ui_in.value)
        dut.ui_in.value = (v | (1 << gpio_bit)) if level else (v & ~(1 << gpio_bit))

    async def pulse(low_units, high_units):
        drive(0)
        await Timer(round(low_units * unit_ns), unit="ns")
        drive(1)
        await Timer(round(high_units * unit_ns), unit="ns")

    drive(1)
    await Timer(20 * unit_ns, unit="ns")
    await pulse(16, 8)  # leader
    for b in data_bytes:
        for i in range(8):  # LSB first
            await pulse(1, 3 if (b >> i) & 1 else 1)
    await pulse(1, 4)  # stop burst, then idle


# ---------------------------------------------------------------------------
# SPI monitor: decodes SCK, MOSI and MISO by the rules of one SPI mode
# ---------------------------------------------------------------------------

class SpiMonitor:
    """Watch SCK = GPIO0, MOSI = GPIO1 and MISO = GPIO2 at the pads.

    The monitor knows the mode and nothing about the programs. It checks
    that SCK rests at the mode's idle level, takes MOSI and MISO on the
    mode's sampling edge, and checks that neither line changes within
    GUARD clock cycles of a sampling edge. A byte is eight sampling edges,
    most significant bit first.
    """

    GUARD = 4  # clock cycles a data line must hold still around a sample

    def __init__(self, dut, mode, sck=0, mosi=1, miso=2):
        self.dut = dut
        self.cpol, self.cpha = (mode >> 1) & 1, mode & 1
        self.pins = (sck, mosi, miso)
        self.mosi_bytes, self.miso_bytes = [], []
        self.errors = []
        self.leading = self.trailing = 0
        self._armed = False
        self._level = (0, 0, 0)
        self._t = 0
        self._changed = [0, 0]      # cycle of the last change: MOSI, MISO
        self._sampled = None        # cycle of the last sampling edge
        self._bits = 0
        self._sh = [0, 0]

    def arm(self):
        """Start decoding. SCK has to be at rest, at the idle level."""
        if self._level[0] != self.cpol:
            self.errors.append("SCK rests at %d, and the mode's idle level is %d"
                               % (self._level[0], self.cpol))
        self._armed = True

    def finish(self):
        """Stop decoding and return the list of errors found."""
        self._armed = False
        if self._level[0] != self.cpol:
            self.errors.append("SCK ends at %d, and the mode's idle level is %d"
                               % (self._level[0], self.cpol))
        if self._bits:
            self.errors.append("%d bits are left over after the last byte" % self._bits)
        if self.leading != self.trailing:
            self.errors.append("%d leading edges and %d trailing edges"
                               % (self.leading, self.trailing))
        return self.errors

    async def run(self):
        dut = self.dut
        while True:
            await RisingEdge(dut.clk)
            self._t += 1
            try:
                pads = int(dut.uio_out.value) & int(dut.uio_oe.value)
            except ValueError:
                continue
            level = tuple((pads >> p) & 1 for p in self.pins)
            prev, self._level = self._level, level
            if not self._armed:
                continue
            for i, name in ((0, "MOSI"), (1, "MISO")):
                if level[i + 1] != prev[i + 1]:
                    self._changed[i] = self._t
                    if self._sampled is not None and self._t - self._sampled < self.GUARD:
                        self.errors.append("%s changed %d cycles after a sampling edge"
                                           % (name, self._t - self._sampled))
            if level[0] == prev[0]:
                continue
            is_leading = level[0] != self.cpol
            if is_leading:
                self.leading += 1
            else:
                self.trailing += 1
            if is_leading != (self.cpha == 0):
                continue
            # The sampling edge: take both data lines as they stand.
            self._sampled = self._t
            for i, name in ((0, "MOSI"), (1, "MISO")):
                if self._t - self._changed[i] < self.GUARD:
                    self.errors.append("%s changed %d cycles before a sampling edge"
                                       % (name, self._t - self._changed[i]))
                self._sh[i] = ((self._sh[i] << 1) | level[i + 1]) & 0xFF
            self._bits += 1
            if self._bits == 8:
                self.mosi_bytes.append(self._sh[0])
                self.miso_bytes.append(self._sh[1])
                self._bits, self._sh = 0, [0, 0]


# ---------------------------------------------------------------------------
# I2C bus with one target device, for reads and writes, with a timing record
# ---------------------------------------------------------------------------

# Minimum times of the I2C-bus specification (NXP UM10204), in ns.
I2C_SPEC_NS = {
    "standard": {"period": 10000, "t_hd_sta": 4000, "t_low": 4700, "t_high": 4000,
                 "t_su_dat": 250, "t_su_sto": 4000, "t_buf": 4700},
    "fast": {"period": 2500, "t_hd_sta": 600, "t_low": 1300, "t_high": 600,
             "t_su_dat": 100, "t_su_sto": 600, "t_buf": 1300},
}


class I2CTarget:
    """Open-drain I2C bus model with one target. SDA = GPIO0, SCL = GPIO1.

    The bus is a wired AND of the DUT's pads and the target's own drivers,
    and a released line reads high. The target acknowledges its address,
    takes written bytes, and serves `read_data` when it is read: it shifts
    a bit out after each falling edge of SCL and stops at the master's
    NACK. With `stretch` it holds SCL low for that many clock cycles before
    each byte it sends. Every edge of the resolved lines is recorded with
    its time, and so is every change of the DUT's own drive on SDA.
    `timing()` measures the bus from that record.
    """

    def __init__(self, dut, addr7=0x52, read_data=(), stretch=0):
        self.dut = dut
        self.addr7 = addr7
        self.read_data = list(read_data)
        self.stretch = stretch
        self.addressed = []     # (address, read bit) of every address byte
        self.written = []       # bytes the master wrote
        self.sent = []          # bytes the master clocked out of the target
        self.master_acks = []   # True for each sent byte the master acknowledged
        self.starts, self.stops = [], []
        self.scl_edges, self.sda_edges = [], []   # (time in ns, new level)
        self.dut_sda_changes = []                 # times the DUT's SDA drive changed
        self.stretch_rises = []                   # rises of SCL that ended a stretch
        self._sda_low = self._scl_low = False
        self._hold = 0
        self._phase = "idle"
        self._bit = self._sh = self._cur = 0
        self._matched = self._ack = False

    def _next_byte(self):
        self._cur = self.read_data.pop(0) if self.read_data else 0xFF
        self._bit = 0
        self._sda_low = not (self._cur >> 7) & 1
        if self.stretch:
            self._scl_low, self._hold = True, self.stretch

    def _on_fall(self):
        if self._phase == "addr":
            if self._bit == 8:
                self.addressed.append((self._sh >> 1, self._sh & 1))
                self._matched = (self._sh >> 1) == self.addr7
                self._sda_low = self._matched
            elif self._bit == 9:
                self._sda_low = False
                if not self._matched:
                    self._phase = "idle"
                elif self._sh & 1:
                    self._phase = "read"
                    self._next_byte()
                else:
                    self._phase, self._bit, self._sh = "write", 0, 0
        elif self._phase == "write":
            if self._bit == 8:
                self.written.append(self._sh)
                self._sda_low = True
            elif self._bit == 9:
                self._sda_low = False
                self._bit, self._sh = 0, 0
        elif self._phase == "read":
            if self._bit == 9:
                self.sent.append(self._cur)
                self.master_acks.append(self._ack)
                if self._ack:
                    self._next_byte()
                else:
                    self._phase, self._sda_low = "idle", False
            elif self._bit == 8:
                self._sda_low = False           # the master's acknowledge slot
            else:
                self._sda_low = not (self._cur >> (7 - self._bit)) & 1

    def _on_rise(self, sda):
        if self._phase in ("addr", "write"):
            if self._bit < 8:
                self._sh = ((self._sh << 1) | sda) & 0xFF
            self._bit += 1
        elif self._phase == "read":
            if self._bit == 8:
                self._ack = sda == 0
            self._bit += 1

    async def run(self):
        import cocotb.utils
        dut = self.dut
        prev_sda, prev_scl, prev_drive = 1, 1, 0
        while True:
            await RisingEdge(dut.clk)
            try:
                out = int(dut.uio_out.value)
                oe = int(dut.uio_oe.value)
            except ValueError:
                out, oe = 0, 0
            released = False
            if self._hold:
                self._hold -= 1
                if not self._hold:
                    self._scl_low, released = False, True
            sda = 0 if ((oe & ~out & 1) or self._sda_low) else 1
            scl = 0 if ((oe & ~out & 2) or self._scl_low) else 1
            now = cocotb.utils.get_sim_time("ns")
            if released and scl:                # the target, not the DUT, let SCL rise
                self.stretch_rises.append(now)
            drive = oe & ~out & 1
            if drive != prev_drive:
                self.dut_sda_changes.append(now)
            prev_drive = drive
            if sda != prev_sda:
                self.sda_edges.append((now, sda))
            if scl != prev_scl:
                self.scl_edges.append((now, scl))
            if scl and prev_scl and sda != prev_sda:
                if not sda:                     # START
                    self.starts.append(now)
                    self._phase, self._bit, self._sh = "addr", 0, 0
                    self._sda_low = False
                else:                           # STOP
                    self.stops.append(now)
                    self._phase, self._sda_low = "idle", False
            elif self._phase != "idle":
                if scl and not prev_scl:
                    self._on_rise(sda)
                elif prev_scl and not scl:
                    self._on_fall()
            dut.uio_in.value = (out & oe & 0xFC) | (scl << 1) | sda
            prev_sda, prev_scl = sda, scl

    def timing(self):
        """The smallest value of each bus time in the record, in ns. A time
        that the record does not contain is absent from the result. The
        data hold time, t_hd_dat, is the DUT's own: from a falling edge of
        SCL to the next change of the DUT's drive on SDA. A clock period
        that begins with the end of a stretch is counted apart, as
        period_after_stretch."""
        marks = sorted(self.starts + self.stops)

        def clear(a, b):                        # no START or STOP inside (a, b)
            return not any(a < m < b for m in marks)

        rises = [t for t, level in self.scl_edges if level]
        falls = [t for t, level in self.scl_edges if not level]
        got = {}

        def keep(name, value):
            if name not in got or value < got[name]:
                got[name] = value

        for t in self.starts:
            after = [f for f in falls if f > t]
            if after:
                keep("t_hd_sta", after[0] - t)
        for f in falls:
            after = [r for r in rises if r > f]
            if after:
                keep("t_low", after[0] - f)
        for r in rises:
            after = [f for f in falls if f > r]
            if after and clear(r, after[0]):
                keep("t_high", after[0] - r)
        for a, b in zip(rises, rises[1:]):
            if clear(a, b):
                keep("period_after_stretch" if a in self.stretch_rises else "period", b - a)
        for t, _ in self.sda_edges:
            after = [r for r in rises if r > t]
            if t not in marks and after:
                keep("t_su_dat", after[0] - t)
        for t in self.dut_sda_changes:
            before = [f for f in falls if f <= t]
            if t not in marks and before and clear(before[-1], t):
                keep("t_hd_dat", t - before[-1])
        for t in self.stops:
            before = [r for r in rises if r < t]
            if before:
                keep("t_su_sto", t - before[-1])
            after = [s for s in self.starts if s > t]
            if after:
                keep("t_buf", after[0] - t)
        return got
