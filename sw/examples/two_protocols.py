# Metastable protocol emulator - two protocols at once
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# Two unrelated bit rates from one chip at the same time: SM0 sends a USB
# low-speed packet on GPIO0 and GPIO1 at 1.5 MHz (divider 4 + 43/256),
# while SM1 drives a WS2812 frame on GPIO4 at 800 kHz (divider
# 6 + 64/256). The two programs are the ones of usb_ls_tx.py and
# ws2812.py, side by side in the instruction memory.
#
# As in usb_ls_tx.py, the USB side needs a host that paces its SPI
# writes, so this file has the programs and their settings and no main().

from metastable_asm import *
import usb_ls_tx
import ws2812

USB_ORIGIN = 0
WS_ORIGIN = 8
WS_PIN = 4

USB_PROG = usb_ls_tx.PROG
WS_PROG = ws2812.program(WS_ORIGIN)


def config():
    """The register writes for both state machines, as (address, value)
    pairs."""
    return usb_ls_tx.config(sm=0) + ws2812.config(sm=1, pin=WS_PIN, origin=WS_ORIGIN)
