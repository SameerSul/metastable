# Metastable protocol emulator - Manchester loopback
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# IEEE 802.3 Manchester coding on GPIO0 (uio[0]): a bit is 8 ticks with a
# transition in the middle, rising for a 1 and falling for a 0. SM0
# transmits. SM1 locks onto the first mid-bit rising edge of an 0xAA
# preamble, then samples the second half of every bit at the
# transmitter's rate. Both run from one clock, so there is no drift.
#
# The transmit loop needs 4 ticks per half bit, so at 50 MHz and divider
# 1 the ceiling is about 6 Mb/s.
#
#   >>> import manchester
#   >>> manchester.main([0x0F, 0x96, 0x55, 0xC3])

from metastable_asm import *

PREAMBLE = 0xAA
OPT = True
LO, HI = side(0, 1, OPT), side(1, 1, OPT)

TX_PROG = [
    SET(SDST_PINDIRS, 1),                  # 0: GPIO0 output, idle low
    OUT(ODST_X, 1),                        # 1: next bit (autopull)
    JMP(5, C_NOTX),                        # 2: 0 -> falling symbol
    MOV(MDST_Y, SRC_Y, side=LO, delay=3),  # 3: '1': low half
    JMP(1, side=HI, delay=1),              # 4:      high half (+1,2)
    MOV(MDST_Y, SRC_Y, side=HI, delay=3),  # 5: '0': high half
    JMP(1, side=LO, delay=1),              # 6:      low half (+1,2)
]

RX0 = 8
RX_PROG = [
    WAIT(1, W_GPIO, 0),                    # 8: first mid-bit rising edge
    IN_(SRC_PINS, 1, delay=6),             # 9: sample 2nd half of bit
    JMP(9),                                # 10: 8 ticks per bit
]


def config(clkdiv=8):
    """The register writes for both state machines, as (address, value)
    pairs. At divider 8 one byte on the wire lasts 512 clock cycles,
    longer than a host FIFO write."""
    return (sm_config(0, div_int=clkdiv, wrap_bottom=0, wrap_top=6,
                      shiftctrl=AUTOPULL,            # MSB first
                      pin_set=0x10,                  # GPIO0 direction
                      pin_side=SIDE_OPT | 0x10)      # GPIO0 value
            + sm_config(1, div_int=clkdiv, wrap_bottom=RX0, wrap_top=10,
                        shiftctrl=AUTOPUSH,          # MSB first
                        pin_in=0))


def setup(host, clkdiv=8):
    host.ctrl(0, 0)
    host.load_program(0, TX_PROG)
    host.load_program(RX0, RX_PROG)
    for addr, value in config(clkdiv):
        host.write(addr, value)
    host.ctrl(0b11, 0b11)


def loopback(host, payload):
    """Send the preamble and the payload, and return what SM1 decoded.
    The receiver free-runs after the frame, so only the first bytes count."""
    assert len(payload) <= 7
    host.tx(0, [PREAMBLE] + list(payload))
    return host.rx(1, len(payload) + 1)[1:]


def main(payload=(0x0F, 0x96, 0x55, 0xC3)):
    from ttboard.demoboard import DemoBoard
    from uart_loopback import make_host

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host)
    got = loopback(host, payload)
    print("sent    ", [hex(b) for b in payload])
    print("received", [hex(b) for b in got])
    assert list(got) == list(payload), "loopback mismatch"
    print("Manchester loopback OK")
    return got
