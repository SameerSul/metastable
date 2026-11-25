# Metastable protocol emulator - UART loopback demo for the TT demo board
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# The same UART program the cocotb suite verifies, for the fabricated chip:
# SM0 transmits 8N1 frames on GPIO0 (uio[0]), SM1 receives on the same pin
# through the pin router, and the bytes come back over SPI.
#
# Runs under MicroPython on the Tiny Tapeout demo board (tt-micropython
# firmware). Copy sw/metastable_asm.py, sw/metastable_host.py and this file
# onto the board, then:
#
#   >>> import uart_loopback
#   >>> uart_loopback.main()
#
# The chip's SPI slave sits on ui[0]=SCK, ui[1]=CS_N, ui[2]=MOSI, uo[0]=MISO.

from metastable_asm import *
from metastable_host import MetastableHost


def make_host(tt):
    """Adapt the ttboard DemoBoard pins to MetastableHost."""

    def drive(sck, cs_n, mosi):
        tt.ui_in.value = (tt.ui_in.value & ~0x07) | sck | (cs_n << 1) | (mosi << 2)

    def sample():
        return tt.uo_out.value & 1

    return MetastableHost(drive, sample)


# UART TX, 8N1, LSB first, 8 ticks/bit, TX = GPIO0 (SET + OUT pins)
TX_PROG = [
    SET(SDST_PINDIRS, 1),             # 0: GPIO0 output
    SET(SDST_PINS, 1),                # 1: line idle high
    PULL(block=1),                    # 2: stall (idle) until data
    SET(SDST_X, 7),                   # 3: bit counter
    SET(SDST_PINS, 0, delay=7),       # 4: start bit, 8 ticks
    OUT(ODST_PINS, 1),                # 5: shift one data bit
    JMP(5, C_XDEC, delay=6),          # 6: 8 ticks per bit
    SET(SDST_PINS, 1, delay=6),       # 7: stop bit, then wrap
]

# UART RX on GPIO0, samples mid-bit
RX0 = 8
RX_PROG = [
    WAIT(0, W_PIN, 0),           # 8: start bit edge
    SET(SDST_X, 7, delay=10),    # 9: align to centre of data bit 0
    IN_(SRC_PINS, 1),            # 10:
    JMP(10, C_XDEC, delay=6),    # 11: 8 ticks per bit
    WAIT(1, W_PIN, 0),           # 12: wait for stop bit (no false start)
    PUSH(block=0),               # 13:
]


def config(clkdiv=2):
    """The register writes for both state machines, as (address, value)
    pairs. SM0 transmits and SM1 receives on the same pin, 8 ticks a bit,
    so the baud rate is clk / (8 * clkdiv)."""
    return (sm_config(0, div_int=clkdiv, wrap_bottom=0, wrap_top=7,
                      pin_out=0x10, pin_set=0x10)          # GPIO0: base 0, count 1
            + sm_config(1, div_int=clkdiv, wrap_bottom=RX0, wrap_top=13, pin_in=0))


def main(payload=(0xA5, 0x3C, 0x00, 0xFF)):
    from ttboard.demoboard import DemoBoard

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)

    host.ctrl(0, 0)
    host.load_program(0, TX_PROG)
    host.load_program(RX0, RX_PROG)
    for addr, value in config():
        host.write(addr, value)

    # start SM0 first so the line is idle high before the receiver arms
    host.ctrl(0b01, 0b01)  # SM0 enable + restart
    host.ctrl(0b11, 0b10)  # SM0 stays enabled, SM1 enable + restart

    host.tx(0, payload)
    got = host.rx(1, len(payload))
    print("sent    ", [hex(b) for b in payload])
    print("received", [hex(b) for b in got])
    assert list(got) == list(payload), "loopback mismatch"
    print("UART loopback OK")
    return got
