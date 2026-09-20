# Metastable protocol emulator - host-side SPI driver
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# Pure Python, no dependencies: runs on CPython and MicroPython (RP2040 on
# the Tiny Tapeout demo board). The chip is a mode-0 SPI slave that needs
# clk >= 8x SCK; a bit-banged host from Python is orders of magnitude
# slower than that, so no pacing is required.
#
# Wiring (chip side): ui[0] = SCK, ui[1] = CS_N, ui[2] = MOSI, uo[0] = MISO.

from metastable_asm import (
    R_CTRL, R_FSTAT, SM, TXF, RXF,
    CLKDIV_INT_L, CLKDIV_INT_H, CLKDIV_FRAC,
    WRAP_TOP, WRAP_BOTTOM,
)


def fstat_bits(fstat, sm):
    """Decode one SM's nibble of FSTAT into a dict of flags."""
    nib = (fstat >> (4 * sm)) & 0xF
    return {
        "tx_empty": bool(nib & 1),
        "tx_full": bool(nib & 2),
        "rx_empty": bool(nib & 4),
        "rx_full": bool(nib & 8),
    }


class MetastableHost:
    """Register-level driver over a caller-supplied pin interface.

    drive(sck, cs_n, mosi): set the three host-driven pins (0/1 each).
    sample():               return the current MISO level (0/1).
    """

    def __init__(self, drive, sample):
        self._drive = drive
        self._sample = sample
        drive(0, 1, 0)  # idle: CS_N high, SCK low

    # ------------------------------------------------------------- SPI core
    def xfer(self, tx_bytes):
        """Mode-0 transfer: send tx_bytes under one CS, return RX bytes."""
        rx = []
        self._drive(0, 0, 0)
        for b in tx_bytes:
            r = 0
            for i in range(7, -1, -1):
                bit = (b >> i) & 1
                self._drive(0, 0, bit)
                r = (r << 1) | (self._sample() & 1)
                self._drive(1, 0, bit)
            rx.append(r)
        self._drive(0, 0, 0)
        self._drive(0, 1, 0)
        return rx

    # ------------------------------------------------------ register access
    def write(self, addr, data):
        """Write one byte or a list of bytes at addr (auto-increment)."""
        if isinstance(data, int):
            data = [data]
        self.xfer([0x80 | addr] + list(data))

    def read(self, addr, n=1):
        """Read n bytes starting at addr (auto-increment)."""
        return self.xfer([addr & 0x7F] + [0] * n)[1:]

    def load_program(self, origin, instrs):
        """Write 16-bit instructions into imem at word address origin."""
        data = []
        for w in instrs:
            data += [w & 0xFF, (w >> 8) & 0xFF]
        self.write(2 * origin, data)

    # ------------------------------------------------------- configuration
    def set_clkdiv(self, sm, div_int, div_frac=0):
        """Divided tick = clk / (div_int + div_frac/256)."""
        base = SM(sm)
        self.write(base + CLKDIV_INT_L, div_int & 0xFF)
        self.write(base + CLKDIV_INT_H, (div_int >> 8) & 0xFF)
        self.write(base + CLKDIV_FRAC, div_frac & 0xFF)

    def set_wrap(self, sm, bottom, top):
        """Program region: restart enters at bottom, wraps top -> bottom."""
        self.write(SM(sm) + WRAP_BOTTOM, bottom)
        self.write(SM(sm) + WRAP_TOP, top)

    def ctrl(self, enable_mask, restart_mask=0):
        """Write CTRL: bit i of enable_mask enables SM i; restart_mask
        restarts SM i (enter at WRAP_BOTTOM, flush its FIFOs)."""
        self.write(R_CTRL, (enable_mask & 3) | ((restart_mask & 3) << 4))

    def enable(self, sm0=False, sm1=False, restart=True):
        """Enable (and by default restart) the given SMs, disable the rest."""
        v = (1 if sm0 else 0) | (2 if sm1 else 0)
        self.ctrl(v, v if restart else 0)

    # ---------------------------------------------------------------- FIFOs
    def fstat(self, sm=None):
        f = self.read(R_FSTAT, 1)[0]
        return f if sm is None else fstat_bits(f, sm)

    def tx(self, sm, data):
        """Push bytes into an SM's TX FIFO, waiting out backpressure."""
        for b in data:
            while self.fstat(sm)["tx_full"]:
                pass
            self.write(SM(sm) + TXF, b)

    def rx(self, sm, n, max_polls=10000):
        """Pop n bytes from an SM's RX FIFO; returns what arrived in time."""
        out = []
        polls = 0
        while len(out) < n and polls < max_polls:
            if self.fstat(sm)["rx_empty"]:
                polls += 1
            else:
                out += self.read(SM(sm) + RXF, 1)
        return out
