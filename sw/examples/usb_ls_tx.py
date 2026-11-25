# Metastable protocol emulator - USB low-speed transmit
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# A USB low-speed packet on D+ = GPIO0 and D- = GPIO1, at 1.5 Mb/s. The
# divider 4 + 43/256 gives 8 ticks a bit, 0.03% above the nominal rate.
# The state machine sends one two-bit symbol (J, K or SE0) every bit
# time. The host builds the packet: sync, NRZI coding, bit stuffing,
# CRC16 and the end of packet are computed here, in packet_symbols().
#
# The state machine takes a FIFO byte, four symbols, every 2.67 us, and
# the FIFO holds eight. A packet is longer than that, so the host has to
# keep up: an SPI write of one FIFO byte at SCK = clk/8 takes about as
# long as the line takes to send it. The bit-banged driver in
# metastable_host.py is far too slow for that. This file therefore has
# the program, its settings and the packet builder, and no main(): on
# the board it needs a host that paces its SPI writes.

from metastable_asm import *

DP, DM = 0, 1
J, K, SE0 = 0b10, 0b01, 0b00    # low speed: J is D- high and D+ low
PID_DATA0 = 0xC3

PROG = [
    SET(SDST_PINS, 2),               # 0: idle J (D- high, D+ low)
    SET(SDST_PINDIRS, 3),            # 1: drive both lines
    OUT(ODST_PINS, 2, delay=6),      # 2: one symbol per 8 ticks
    JMP(2),                          # 3:
]


def config(sm=0):
    """The register writes, as (address, value) pairs."""
    return sm_config(sm, div_int=4, div_frac=43,       # 1.5 MHz bit clock
                     wrap_bottom=0, wrap_top=3,
                     shiftctrl=AUTOPULL | OUT_RIGHT,
                     pin_set=0x20,                     # base 0, count 2
                     pin_out=0x20)                     # base 0, count 2


def crc16(data):
    """CRC16-USB (poly 0x8005 reflected, init 0xFFFF), complemented."""
    crc = 0xFFFF
    for b in data:
        for i in range(8):
            if (crc ^ (b >> i)) & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return (~crc) & 0xFFFF


def packet_symbols(pid, payload):
    """Build one LS packet as a J/K/SE0 symbol list: sync + PID (+ data +
    CRC16) with NRZI encoding ('0' toggles, '1' holds) and bit stuffing
    (a 0 forced after six 1s), then the SE0/SE0/J EOP."""
    packet = [pid] + list(payload)
    if payload:
        crc = crc16(payload)
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
    symbols, state = [], J  # bus idles at J
    for bit in stuffed:
        if not bit:
            state = K if state == J else J
        symbols.append(state)
    return symbols + [SE0, SE0, J]


def symbols_to_bytes(symbols, lead_idle=8):
    """Pack symbols 4 per FIFO byte for OUT PINS,2 shifting right, with a
    leading idle-J run and the final byte padded with idle J."""
    syms = [J] * lead_idle + list(symbols)
    while len(syms) % 4:
        syms.append(J)
    return [
        syms[i] | (syms[i + 1] << 2) | (syms[i + 2] << 4) | (syms[i + 3] << 6)
        for i in range(0, len(syms), 4)
    ]


def packet_bytes(pid, payload):
    """The FIFO bytes of one packet: stream them into the TX FIFO."""
    return symbols_to_bytes(packet_symbols(pid, payload))
