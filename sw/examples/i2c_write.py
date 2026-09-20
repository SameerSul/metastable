# Metastable protocol emulator - I2C master write demo for the TT demo board
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# The same open-drain I2C master the cocotb suite verifies against an
# ACKing slave model, run on silicon. SDA = uio[0], SCL = uio[1]; both
# need external pull-ups (2.2-10 kOhm to 3.3 V). Pin output latches stay
# at 0 and the program only toggles pin *directions*: dir=1 drives low,
# dir=0 releases to the pull-up — true open drain.
#
#   >>> import i2c_write
#   >>> i2c_write.main(0x52, [0x57])   # write one byte to device 0x52

from metastable_asm import *
from metastable_host import MetastableHost, fstat_bits
from uart_loopback import make_host

OPT = True
LO = side(1, 1, OPT)  # SCL dir=1: drive low
HI = side(0, 1, OPT)  # SCL dir=0: release high

# Instruction 3 (SET Y, n-1) is patched per transaction with the byte count.
SET_Y_ADDR = 3

PROG = [
    SET(SDST_PINDIRS, 0),                    # 0: SDA released (bus idle)
    PULL(block=1),                           # 1: wait for first byte
    SET(SDST_PINDIRS, 1, delay=7),           # 2: START: SDA low, SCL high
    SET(SDST_Y, 1),                          # 3: byte count - 1 (patched)
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


def setup(host, clkdiv=63):
    """clkdiv 63 at 50 MHz gives ~99 kHz SCL (8 ticks per bit)."""
    host.load_program(0, PROG)
    host.set_clkdiv(0, clkdiv)
    host.set_wrap(0, 0, 13)
    host.write(SM(0) + SHIFTCTRL, AUTOPULL)  # shift left, MSB first
    host.write(SM(0) + PIN_OUT, 0x10)        # SDA: base 0, count 1
    host.write(SM(0) + PIN_SET, 0x10)        # SDA: base 0, count 1
    host.write(SM(0) + PIN_IN, 0)            # SDA
    # SCL: base 1, count 1, optional side-set, drives pindirs
    host.write(SM(0) + PIN_SIDE, SIDE_PINDIR | SIDE_OPT | 0x11)


def write_txn(host, addr7, data):
    """One I2C write transaction; returns the ACK bits (0 = all ACKed)."""
    wire = [(addr7 << 1) & 0xFE] + list(data)
    assert len(wire) <= 8, "one PUSH holds at most 8 ACK bits"
    # patch the byte count, then restart so the SM re-reads it
    host.load_program(SET_Y_ADDR, [SET(SDST_Y, len(wire) - 1)])
    host.ctrl(0b01, 0b01)
    host.tx(0, [~b & 0xFF for b in wire])  # dir=1 pulls the line low
    acks = host.rx(0, 1)
    return acks[0] if acks else None


def main(addr7=0x52, data=(0x57,)):
    from ttboard.demoboard import DemoBoard

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host)
    acks = write_txn(host, addr7, data)
    if acks == 0:
        print("write to 0x%02x OK (all bytes ACKed)" % addr7)
    else:
        print("ACK bits 0x%02x — NAK from device 0x%02x" % (acks, addr7))
    return acks
