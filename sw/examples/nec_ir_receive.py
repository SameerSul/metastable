# Metastable protocol emulator - NEC infrared receive
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# Decode an NEC infrared frame from a demodulating receiver on GPIO8
# (ui[3], input only): low during a burst, high in a gap. The program
# measures the length of each gap. A bit is: wait for the burst, wait for
# its end, then take one sample a fixed delay later. The sample falls
# inside the next burst for a 0 and inside the gap for a 1. Timing from
# the end of the burst makes the loop correct itself for both bit
# lengths. The leader is passed over without a sample, so exactly 32 bits
# arrive as 4 bytes: address, its complement, command, its complement.
#
# The line has to idle high before the state machine starts, or the
# first WAIT takes the low line for a burst.
#
#   >>> import nec_ir_receive
#   >>> nec_ir_receive.main()      # then press a button on the remote

from metastable_asm import *

IR_PIN = 8          # GPIO8 = ui[3]
UNIT_US = 562.5     # the NEC time unit: a burst is one unit

PROG = [
    WAIT(0, W_PIN, 0),            # 0: leader burst
    WAIT(1, W_PIN, 0),            # 1: leader gap begins
    SET(SDST_X, 31),              # 2: 32 data bits
    WAIT(0, W_PIN, 0),            # 3: bit burst
    WAIT(1, W_PIN, 0, delay=20),  # 4: sample 21 ticks after the burst ends:
                                  #    mid-burst for '0', mid-gap for '1'
    IN_(SRC_PINS, 1),             # 5: high = long gap = '1'
    JMP(3, C_XDEC),               # 6:
    JMP(7),                       # 7: park
]


def config(clkdiv=2000):
    """The register writes, as (address, value) pairs. Divider 2000 at
    50 MHz is a 40 us tick, which puts the sample 840 us after a burst
    ends: 1.5 units, between the short gap and the long one."""
    return sm_config(0, div_int=clkdiv, wrap_bottom=0, wrap_top=7,
                     shiftctrl=AUTOPUSH | IN_RIGHT,     # LSB first
                     pin_in=IR_PIN)


def setup(host, clkdiv=2000):
    host.ctrl(0, 0)
    host.load_program(0, PROG)
    for addr, value in config(clkdiv):
        host.write(addr, value)


def receive(host, max_polls=100000):
    """Arm the receiver and wait for one frame. Returns (address,
    command), or None when the frame does not arrive or fails its
    complement check."""
    host.ctrl(0b01, 0b01)
    got = host.rx(0, 4, max_polls)
    if len(got) != 4 or got[0] ^ got[1] != 0xFF or got[2] ^ got[3] != 0xFF:
        return None
    return got[0], got[2]


def main():
    from ttboard.demoboard import DemoBoard
    from uart_loopback import make_host

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host)
    frame = receive(host)
    if frame is None:
        print("no valid NEC frame")
    else:
        print("address 0x%02x command 0x%02x" % frame)
    return frame
