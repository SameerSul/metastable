# Metastable protocol emulator - capture / replay for the TT demo board
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# The suite-verified capture/replay programs on silicon: SM0 samples
# GPIO4-7 (IN PINS,4) into a snapshot that freezes itself when the RX
# FIFO fills; SM1 replays a 4-bit waveform from its FIFO (OUT PINS,4).
# Use them separately (probe an external bus / generate a stimulus) or
# together as a self-test.
#
#   >>> import capture_replay
#   >>> capture_replay.replay(host, [1, 3, 7, 0xF] * 4, div=16)
#   >>> capture_replay.capture(host, div=16)   # 16-sample snapshot

from metastable_asm import *
from metastable_host import MetastableHost
from uart_loopback import make_host

CAP_PROG = [IN_(SRC_PINS, 4), JMP(0)]            # at 0: 2 ticks/sample
AWG0 = 8
AWG_PROG = [SET(SDST_PINDIRS, 0xF),              # at 8: GPIO4-7 outputs
            OUT(ODST_PINS, 4), JMP(AWG0 + 1)]    # 2 ticks/symbol


def config(div=16):
    """The register writes for both state machines, as (address, value)
    pairs: SM0 samples GPIO4-7 and SM1 replays onto them, at one rate."""
    return (sm_config(0, div_int=div, wrap_bottom=0, wrap_top=31,
                      shiftctrl=AUTOPUSH,          # 2 samples/byte, MSB first
                      pin_in=4)
            + sm_config(1, div_int=div, wrap_bottom=AWG0, wrap_top=31,
                        shiftctrl=AUTOPULL,        # 2 symbols/byte, MSB first
                        pin_out=0x44,              # base 4, count 4
                        pin_set=0x44))


def setup(host, div=16):
    host.load_program(0, CAP_PROG)
    host.load_program(AWG0, AWG_PROG)
    for addr, value in config(div):
        host.write(addr, value)


def replay(host, nibbles, div=16):
    """Stream a waveform (list of 0..15) out of GPIO4-7."""
    setup(host, div)
    if len(nibbles) % 2:
        nibbles = list(nibbles) + [nibbles[-1]]
    host.ctrl(0b10, 0b10)
    host.tx(1, [(nibbles[i] << 4) | nibbles[i + 1]
                for i in range(0, len(nibbles), 2)])


def capture(host, div=16):
    """Arm a 16-sample snapshot of GPIO4-7; returns the sample list."""
    setup(host, div)
    host.ctrl(0b01, 0b01)  # runs until the RX FIFO fills, then freezes
    raw = host.rx(0, 8)
    host.ctrl(0b00)
    samples = []
    for b in raw:
        samples += [b >> 4, b & 0xF]
    return samples


def main():
    from ttboard.demoboard import DemoBoard

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    wave = [1, 3, 7, 0xF, 0xE, 0xC, 8, 0] * 2
    replay(host, wave)
    print("captured:", capture(host))
