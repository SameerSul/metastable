# Metastable protocol emulator - SPI master and slave in modes 0 to 3
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# An SPI mode is two bits. CPOL is the level of SCK between bytes. CPHA
# says which clock edge samples the data: the leading edge (the one that
# leaves the idle level) when it is 0, the trailing edge when it is 1.
#
#   mode  CPOL  CPHA   SCK idles   data sampled on   data changes on
#    0     0     0     low         rising edge       falling edge
#    1     0     1     low         falling edge      rising edge
#    2     1     0     high        falling edge      rising edge
#    3     1     1     high        rising edge       falling edge
#
# The master and the slave are programs, so one chip speaks all four. The
# cocotb suite runs each mode: the master on SM0, the slave on SM1, and a
# monitor that decodes the three wires by the rules of that mode.
#
# Pins, as the suite and the demo wire them: SCK = GPIO0, MOSI = GPIO1,
# MISO = GPIO2. Bytes go most significant bit first.
#
#   >>> import spi_modes
#   >>> spi_modes.main(3)          # loop four bytes through mode 3

from metastable_asm import *

SCK, MOSI, MISO = 0, 1, 2
MASTER_ORIGIN = 0   # the master's first instruction
SLAVE_ORIGIN = 8    # the slave's first instruction


def cpol(mode):
    return (mode >> 1) & 1


def cpha(mode):
    return mode & 1


def master_program(mode):
    """The SPI master for one mode: six instructions, four ticks a bit.

    SCK is the side-set pin, so every instruction states its level. The
    master waits for a byte at the PULL, with SCK at its idle level. A
    bit is two ticks at one level and two at the other.
    """
    assert 0 <= mode <= 3
    idle = side(cpol(mode), 1)        # SCK between bytes
    lead = side(1 - cpol(mode), 1)    # SCK after the leading edge
    # CPHA 0: MOSI changes as SCK returns to idle, and the leading edge
    # samples. CPHA 1: MOSI changes on the leading edge, and the trailing
    # edge samples.
    shift, sample = (idle, lead) if cpha(mode) == 0 else (lead, idle)
    return [
        SET(SDST_PINDIRS, 3, side=idle),          # 0: SCK and MOSI drive
        PULL(block=1, side=idle),                 # 1: wait for a byte
        SET(SDST_X, 7, side=idle),                # 2: eight bits
        OUT(ODST_PINS, 1, side=shift, delay=1),   # 3: MOSI <= next bit
        IN_(SRC_PINS, 1, side=sample),            # 4: sample MISO
        JMP(3, C_XDEC, side=sample),              # 5: next bit
    ]


def slave_program(mode):
    """The SPI slave for one mode: five or six instructions, paced by SCK.

    It has no chip select: every eight clocks are a byte. Give it a tick
    at least eight times faster than the master's. Every pass begins by
    waiting for SCK at its idle level. Without that wait, a slave that
    starts before the master has raised an idle-high SCK takes the low
    line for a clock edge and is one bit out for good.
    """
    assert 0 <= mode <= 3
    idle, lead = cpol(mode), 1 - cpol(mode)
    if cpha(mode) == 0:
        return [
            SET(SDST_PINDIRS, 1),          # 0: MISO drives
            WAIT(idle, W_GPIO, SCK),       # 1: SCK idle: trailing edge
            OUT(ODST_PINS, 1),             # 2: MISO <= next bit
            WAIT(lead, W_GPIO, SCK),       # 3: leading edge
            IN_(SRC_PINS, 1),              # 4: sample MOSI
        ]
    return [
        SET(SDST_PINDIRS, 1),              # 0: MISO drives
        WAIT(idle, W_GPIO, SCK),           # 1: SCK idle
        WAIT(lead, W_GPIO, SCK),           # 2: leading edge
        OUT(ODST_PINS, 1),                 # 3: MISO <= next bit
        WAIT(idle, W_GPIO, SCK),           # 4: trailing edge
        IN_(SRC_PINS, 1),                  # 5: sample MOSI
    ]


def master_config(sm=0, clkdiv=8, origin=MASTER_ORIGIN):
    """The master's register writes, as (address, value) pairs."""
    return sm_config(sm, div_int=clkdiv, wrap_bottom=origin, wrap_top=origin + 5,
                     shiftctrl=AUTOPUSH,           # shift left, explicit PULL
                     pin_out=0x10 | MOSI,          # MOSI: one pin
                     pin_set=0x20 | SCK,           # SCK and MOSI directions
                     pin_in=MISO,
                     pin_side=0x10 | SCK)          # SCK: one side-set pin


def slave_config(mode, sm=1, clkdiv=1, origin=SLAVE_ORIGIN):
    """The slave's register writes for one mode, as (address, value) pairs."""
    return sm_config(sm, div_int=clkdiv, wrap_bottom=origin,
                     wrap_top=origin + len(slave_program(mode)) - 1,
                     shiftctrl=AUTOPULL | AUTOPUSH,   # shift left
                     pin_out=0x10 | MISO,             # MISO: one pin
                     pin_set=0x10 | MISO,             # MISO direction
                     pin_in=MOSI)


def setup(host, mode, clkdiv=8):
    """Load both programs for one mode and start them: master on SM0,
    slave on SM1. SCK then rests at the mode's idle level."""
    host.ctrl(0, 0)
    host.load_program(MASTER_ORIGIN, master_program(mode))
    host.load_program(SLAVE_ORIGIN, slave_program(mode))
    for addr, value in master_config(clkdiv=clkdiv) + slave_config(mode):
        host.write(addr, value)
    host.ctrl(0b11, 0b11)


def exchange(host, to_slave, to_master):
    """Send to_slave from the master while the slave answers to_master.
    Returns (what the master received, what the slave received)."""
    assert len(to_slave) == len(to_master) <= 8
    host.tx(1, to_master)   # the slave's answers first: it shifts on SCK
    host.tx(0, to_slave)
    return host.rx(0, len(to_slave)), host.rx(1, len(to_slave))


def main(mode=0, to_slave=(0x9E, 0x01, 0x55, 0xC3), to_master=(0xEF, 0x40, 0xAA, 0x81)):
    """Loop four bytes through one mode. Like the UART loopback example,
    it needs no wiring: it relies on each pad reading back the level that
    the chip drives on it, so the slave sees SCK and MOSI, and the master
    sees MISO."""
    from ttboard.demoboard import DemoBoard
    from uart_loopback import make_host

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host, mode)
    m_rx, s_rx = exchange(host, to_slave, to_master)
    print("mode %d: master got %s, slave got %s"
          % (mode, [hex(b) for b in m_rx], [hex(b) for b in s_rx]))
    assert list(m_rx) == list(to_master) and list(s_rx) == list(to_slave), "mismatch"
    print("SPI mode %d OK" % mode)
    return m_rx, s_rx
