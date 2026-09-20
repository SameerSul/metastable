# Metastable protocol emulator - ISA assembler + register map
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# Pure Python, no dependencies: runs on CPython (cocotb testbench) and
# MicroPython (RP2040 host on the Tiny Tapeout demo board). The cocotb
# suite imports this same file, so the assembler the tests verify is the
# assembler you ship to the board.

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
R_FLEVEL0, R_FLEVEL1 = 0x48, 0x49  # [3:0] TX level, [7:4] RX level

def SM(n):  # per-SM register base
    return 0x50 + 0x10 * n

CLKDIV_INT_L, CLKDIV_INT_H, CLKDIV_FRAC = 0x0, 0x1, 0x2
WRAP_TOP, WRAP_BOTTOM, SHIFTCTRL, THRESH = 0x3, 0x4, 0x5, 0x6
PIN_OUT, PIN_SET, PIN_IN, PIN_SIDE, JMP_PIN = 0x7, 0x8, 0x9, 0xA, 0xB
TXF, RXF, STATUS_CFG = 0xC, 0xD, 0xE

# STATUS_CFG bits: [3:0] level N, [4] select RX (default TX)
STATUS_RX = 0x10

# SHIFTCTRL bits
AUTOPULL, AUTOPUSH, OUT_RIGHT, IN_RIGHT = 1, 2, 4, 8

# PIN_SIDE bits: [3:0] base, [5:4] count, [6] optional, [7] drive pindirs
SIDE_OPT, SIDE_PINDIR = 0x40, 0x80

# CTRL bits: [1:0] SM enable, [5:4] SM restart (restart also flushes FIFOs)
CTRL_EN0, CTRL_EN1, CTRL_RST0, CTRL_RST1 = 0x01, 0x02, 0x10, 0x20
