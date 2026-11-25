# Metastable protocol emulator - PS/2 device
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# The chip as a PS/2 device, such as a keyboard: DATA on GPIO0 (uio[0])
# and a clock that the device generates on GPIO1 (uio[1]), at 12.5 kHz.
# A frame is 11 bits: start, 8 data bits least significant first, odd
# parity, stop. It spans two FIFO bytes, and autopull refills in the
# middle of the frame. The host of the bus samples DATA on the falling
# edge of the clock.
#
#   >>> import ps2_device
#   >>> ps2_device.main([0x1C, 0xF0])     # 'A' make, then the break prefix

from metastable_asm import *

DATA, CLK = 0, 1
H, L = side(1, 1), side(0, 1)

PROG = [
    SET(SDST_PINDIRS, 3, side=H),        # 0: DATA + CLK outputs
    SET(SDST_PINS, 3, side=H),           # 1: idle high
    PULL(block=1, side=H),               # 2: wait for a frame
    SET(SDST_X, 10, side=H),             # 3: 11 bits
    OUT(ODST_PINS, 1, delay=7, side=H),  # 4: data setup, CLK high
    JMP(4, C_XDEC, delay=7, side=L),     # 5: CLK low, host samples
    JMP(2, side=H),                      # 6: frame done, CLK high
]


def config(clkdiv=250):
    """The register writes, as (address, value) pairs. Divider 250 at
    50 MHz is a 5 us tick: 16 ticks a bit, 80 us, 12.5 kHz."""
    return sm_config(0, div_int=clkdiv, wrap_bottom=0, wrap_top=6,
                     shiftctrl=AUTOPULL | OUT_RIGHT,
                     pin_out=0x10 | DATA,          # DATA: one pin
                     pin_set=0x20 | DATA,          # DATA and CLK
                     pin_side=0x10 | CLK)          # CLK: one pin


def frame_bytes(value):
    """Pack one PS/2 device frame (start, 8 data LSB first, odd parity,
    stop) into two FIFO bytes for OUT PINS,1 shifting right."""
    parity = 1 ^ (bin(value).count("1") & 1)
    bits = [0] + [(value >> i) & 1 for i in range(8)] + [parity, 1]
    bits += [1] * 5  # pad the second byte with idle-high bits
    return [sum(bits[k + i] << i for i in range(8)) for k in (0, 8)]


def setup(host, clkdiv=250):
    host.ctrl(0, 0)
    host.load_program(0, PROG)
    for addr, value in config(clkdiv):
        host.write(addr, value)
    host.ctrl(0b01, 0b01)


def send(host, codes):
    """Send each byte as one frame. Both FIFO bytes of a frame go in one
    SPI write, so the second is there when the frame needs it."""
    for code in codes:
        while not host.fstat(0)["tx_empty"]:
            pass
        host.write(SM(0) + TXF, frame_bytes(code))


def main(codes=(0x1C, 0xF0)):
    from ttboard.demoboard import DemoBoard
    from uart_loopback import make_host

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host)
    send(host, codes)
    print("sent", [hex(c) for c in codes])
