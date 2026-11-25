# Metastable protocol emulator - WS2812 (NeoPixel) demo for the TT demo board
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# The same WS2812 program the cocotb suite verifies with a pulse-width
# checker: SM0 drives 800 kHz NeoPixel data on GPIO0 (uio[0]).
# Wire the LED strip's DIN to uio[0] (level-shift to 5 V for long strips),
# common ground, then:
#
#   >>> import ws2812
#   >>> ws2812.main([(255, 0, 0), (0, 0, 255)])   # red, blue

from metastable_asm import *
from metastable_host import MetastableHost
from uart_loopback import make_host

S0, S1 = side(0, 1), side(1, 1)

# 8 MHz tick (divider 6+64/256), 10 ticks per bit:
# '0' = 3 ticks high / 7 low, '1' = 7 high / 3 low.
def program(origin=0):
    """The six instructions, for a program that starts at `origin`."""
    return [
        SET(SDST_PINDIRS, 1),                        # +0: once: the pin drives
        OUT(ODST_X, 1, delay=2, side=S0),            # +1: bitloop: low tail (T3)
        JMP(origin + 4, C_NOTX, delay=2, side=S1),   # +2: high leader (T1)
        JMP(origin + 1, delay=3, side=S1),           # +3: '1': stay high (T2)
        SET(SDST_PINDIRS, 1, delay=1, side=S0),      # +4: '0': low (pindir refresh)
        JMP(origin + 1, delay=1, side=S0),           # +5: '0': low, next bit
    ]


PROG = program()


def config(sm=0, pin=0, origin=0):
    """The register writes, as (address, value) pairs: an 8 MHz tick
    from the 50 MHz clock, most significant bit first, data on `pin`."""
    return sm_config(sm, div_int=6, div_frac=64,            # 50 MHz / 6.25
                     wrap_bottom=origin, wrap_top=origin + 5,
                     shiftctrl=AUTOPULL,                     # 8-bit refill
                     pin_set=0x10 | pin,                     # one pin
                     pin_side=0x10 | pin)                    # always on


def setup(host):
    host.ctrl(0, 0)
    host.load_program(0, PROG)
    for addr, value in config():
        host.write(addr, value)
    host.enable(sm0=True)


def show(host, colors):
    """colors: list of (r, g, b) tuples; WS2812 wants GRB, MSB first.

    The frame is loaded while the state machine is stopped, and then the
    state machine starts. A gap between two bytes would read as a latch,
    and a bit-banged host is far slower than the 1.25 us of a bit. One
    call therefore takes at most 8 bytes, the depth of the FIFO: two
    LEDs. A host that paces its SPI writes can stream a longer strip.
    """
    data = []
    for r, g, b in colors:
        data += [g & 0xFF, r & 0xFF, b & 0xFF]
    assert len(data) <= 8, "one call takes at most two LEDs"
    host.ctrl(0b00, 0b01)            # stopped at the first instruction, FIFO empty
    host.write(SM(0) + TXF, data)    # one burst: the address holds on the FIFO
    host.ctrl(0b01, 0b00)            # start
    # latch happens by itself: the OUT stalls low once the FIFO drains


def main(colors=((255, 0, 0), (0, 0, 255))):
    from ttboard.demoboard import DemoBoard

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host)
    show(host, colors)
    print("sent", len(colors), "LEDs")
