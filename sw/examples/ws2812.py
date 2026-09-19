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
PROG = [
    SET(SDST_PINDIRS, 1),                    # 0: once: GPIO0 output
    OUT(ODST_X, 1, delay=2, side=S0),        # 1: bitloop: low tail (T3)
    JMP(4, C_NOTX, delay=2, side=S1),        # 2: high leader (T1)
    JMP(1, delay=3, side=S1),                # 3: '1': stay high (T2)
    SET(SDST_PINDIRS, 1, delay=1, side=S0),  # 4: '0': low (pindir refresh)
    JMP(1, delay=1, side=S0),                # 5: '0': low, next bit
]


def setup(host):
    host.load_program(0, PROG)
    host.set_clkdiv(0, 6, 64)          # 50 MHz / 6.25 = 8 MHz tick
    host.set_wrap(0, 0, 5)
    host.write(SM(0) + SHIFTCTRL, AUTOPULL)  # MSB first, 8-bit refill
    host.write(SM(0) + PIN_SET, 0x10)  # base 0, count 1
    host.write(SM(0) + PIN_SIDE, 0x10) # base 0, count 1, always on
    host.enable(sm0=True)


def show(host, colors):
    """colors: list of (r, g, b) tuples; WS2812 wants GRB, MSB first."""
    data = []
    for r, g, b in colors:
        data += [g & 0xFF, r & 0xFF, b & 0xFF]
    host.tx(0, data)
    # latch happens by itself: the OUT stalls low once the FIFO drains


def main(colors=((255, 0, 0), (0, 255, 0), (0, 0, 255))):
    from ttboard.demoboard import DemoBoard

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host)
    show(host, colors)
    print("sent", len(colors), "LEDs")
