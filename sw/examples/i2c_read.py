# Metastable protocol emulator - I2C master read
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# An open-drain I2C master that reads bytes from a device: START, the
# address with the read bit, then each byte, acknowledged except the
# last, then STOP. SDA = uio[0], SCL = uio[1]; both need external
# pull-ups (2.2-10 kOhm to 3.3 V). As in i2c_write.py, the pin output
# latches stay at 0 and the program toggles pin directions: direction 1
# drives the line low, direction 0 releases it to the pull-up.
#
# Three things the program does on the chip, not on the host:
#
#   - It waits for SCL to read high after every release, so a device that
#     stretches the clock is waited for. The high time then still lasts
#     6 ticks from the moment the master sees the line high.
#   - It tests the address acknowledge with JMP PIN. A device that does
#     not answer gets a STOP at once, and no data clock.
#   - It raises interrupt flag 0 when the transaction ends, either way.
#
# A bit is 15 ticks: SCL low for 8 and high for 7. That split meets the
# low and high times of both standard mode and fast mode, so one program
# serves both with only the divider changed:
#
#   speed       divider at 50 MHz   SCL          SCL low   SCL high
#   standard    33 + 86/256         99.99 kHz    5.33 us   4.67 us
#   fast        8 + 86/256          399.9 kHz    1.33 us   1.17 us
#
# One limit comes from the hardware: a state machine sees its pins once a
# tick. After a stretch, the master can notice the released line up to a
# tick late, so that one clock period can be up to a tick shorter than the
# others, 9.4 us at the standard divider. Its low and high times still
# meet the minimums. The suite measures both times and the shortened
# period. By the same arithmetic, a divider one fourteenth larger
# (35 + 183/256, or 8 + 238/256) keeps every period above the nominal
# one, at 93 kHz and 373 kHz. The suite does not run those two dividers.
#
#   >>> import i2c_read
#   >>> i2c_read.main(0x52, 2)     # read two bytes from device 0x52

from metastable_asm import *

SDA, SCL = 0, 1
DONE_IRQ = 0            # the flag that the program raises at the end
STANDARD = (33, 86)     # (integer, fraction) of the divider: 100 kHz
FAST = (8, 86)          # 400 kHz

OPT = True
LO = side(1, 1, OPT)    # SCL direction 1: drive low
HI = side(0, 1, OPT)    # SCL direction 0: release

# Instruction 3 (SET Y, n - 1) is patched per transaction with the number
# of bytes to read. FIN and ACK are the two jump targets.
SET_Y_ADDR = 3
FIN, ACK = 19, 23
WRAP_TOP_ADDR = 22

PROG = [
    SET(SDST_PINDIRS, 0),                         # 0: SDA released: bus idle
    PULL(block=1),                                # 1: wait for the address byte
    SET(SDST_PINDIRS, 1, delay=5),                # 2: START: SDA low, SCL high
    SET(SDST_Y, 0),                               # 3: bytes to read - 1 (patched)
    SET(SDST_X, 7, side=LO),                      # 4: SCL low, 8 address bits
    OUT(ODST_PINDIRS, 1, delay=6),                # 5: SDA <= address bit
    WAIT(1, W_GPIO, SCL, side=HI, delay=5),       # 6: SCL high, device samples
    JMP(5, C_XDEC, side=LO),                      # 7: SCL low, next bit
    SET(SDST_PINDIRS, 0, delay=6),                # 8: release SDA for the ACK
    WAIT(1, W_GPIO, SCL, side=HI, delay=4),       # 9: SCL high
    JMP(FIN, C_PIN),                              # 10: SDA high: no device, stop
    SET(SDST_X, 7, side=LO),                      # 11: SCL low, 8 data bits
    SET(SDST_PINDIRS, 0, delay=6),                # 12: SDA released: device drives
    WAIT(1, W_GPIO, SCL, side=HI, delay=4),       # 13: SCL high
    IN_(SRC_PINS, 1),                             # 14: sample SDA (autopush at 8)
    JMP(12, C_XDEC, side=LO),                     # 15: SCL low, next bit
    JMP(ACK, C_YDEC),                             # 16: more bytes: acknowledge
    SET(SDST_PINDIRS, 0, delay=5),                # 17: last byte: SDA high, NACK
    WAIT(1, W_GPIO, SCL, side=HI, delay=5),       # 18: SCL high
    IRQ(DONE_IRQ, side=LO),                       # 19: FIN: SCL low, flag raised
    SET(SDST_PINDIRS, 1, delay=6),                # 20: SDA low ahead of STOP
    WAIT(1, W_GPIO, SCL, side=HI, delay=5),       # 21: SCL high
    SET(SDST_PINDIRS, 0, delay=6),                # 22: STOP: SDA rises; wrap to 0
    SET(SDST_PINDIRS, 1, delay=5),                # 23: ACK: SDA low
    WAIT(1, W_GPIO, SCL, side=HI, delay=5),       # 24: SCL high
    JMP(11),                                      # 25: next byte
]


def config(speed=STANDARD, sm=0):
    """The register writes for one speed, as (address, value) pairs."""
    div_int, div_frac = speed
    return sm_config(sm, div_int=div_int, div_frac=div_frac,
                     wrap_bottom=0, wrap_top=WRAP_TOP_ADDR,
                     shiftctrl=AUTOPUSH,           # shift left, most significant bit first
                     pin_out=0x10 | SDA,           # SDA: one pin
                     pin_set=0x10 | SDA,
                     pin_in=SDA,
                     jmp_pin=SDA,
                     # SCL: one pin, optional side-set, on the pin directions
                     pin_side=SIDE_PINDIR | SIDE_OPT | 0x10 | SCL)


def address_byte(addr7):
    """What the host puts in the TX FIFO: the address and the read bit,
    inverted, because direction 1 pulls the line low."""
    return ~((addr7 << 1) | 1) & 0xFF


def count_patch(n):
    """The instruction for SET_Y_ADDR that makes the program read n bytes."""
    assert 1 <= n <= 32
    return SET(SDST_Y, n - 1)


def setup(host, speed=STANDARD):
    host.ctrl(0, 0)
    host.load_program(0, PROG)
    for addr, value in config(speed):
        host.write(addr, value)
    host.ctrl(0b01, 0b01)


def read_txn(host, addr7, n, max_polls=10000):
    """Read n bytes (at most 8, the depth of the RX FIFO) from the device
    at addr7. Returns the bytes, or None when no device acknowledged."""
    assert 1 <= n <= 8
    host.load_program(SET_Y_ADDR, [count_patch(n)])   # the SM waits at the PULL
    host.write(R_IRQ, 1 << DONE_IRQ)                  # clear the flag
    host.tx(0, [address_byte(addr7)])
    for _ in range(max_polls):
        if host.read(R_IRQ, 1)[0] & (1 << DONE_IRQ):
            break
    host.write(R_IRQ, 1 << DONE_IRQ)
    got = host.read(R_FLEVEL0, 1)[0] >> 4             # bytes in the RX FIFO
    if got == 0:
        return None
    return host.rx(0, got)


def main(addr7=0x52, n=2, speed=STANDARD):
    from ttboard.demoboard import DemoBoard
    from uart_loopback import make_host

    tt = DemoBoard.get()
    tt.shuttle.tt_um_sulems6_metastable.enable()
    host = make_host(tt)
    setup(host, speed)
    data = read_txn(host, addr7, n)
    if data is None:
        print("no acknowledge from device 0x%02x" % addr7)
    else:
        print("read from 0x%02x: %s" % (addr7, [hex(b) for b in data]))
    return data
