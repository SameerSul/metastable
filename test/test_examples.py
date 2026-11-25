# Metastable protocol emulator - the examples' own host code, on the simulated chip
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
"""Run each example of sw/examples the way a user runs it on the demo board.

The examples import ttboard.demoboard. Here that name is the stand-in in
test/standin, which puts the simulated design behind the board's pins. A
test of this file therefore runs three things together that test.py does
not: the example's own host function, the driver of sw/metastable_host.py,
and the program on the chip.

The stand-in's host is slow on purpose. A real host bit-bangs its SPI
pins, and it is far slower than any protocol here. An example that only
works when the host keeps up with the line fails in this file.

usb_ls_tx.py and two_protocols.py have no test here. They need a host
that paces its SPI writes, and they have no host function to run.
"""
import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.task import bridge
from cocotb.triggers import ClockCycles, RisingEdge, Timer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "standin"))

from metastable import (  # noqa: E402  (also puts sw/ and sw/examples on the path)
    uio_loopback, I2CSlave, I2CTarget, WS2812Decoder, Ps2Host,
)
from ttboard import demoboard  # noqa: E402
import capture_replay  # noqa: E402
import i2c_read  # noqa: E402
import i2c_write  # noqa: E402
import manchester  # noqa: E402
import nec_ir_receive  # noqa: E402
import ps2_device  # noqa: E402
import spi_modes  # noqa: E402
import uart_loopback  # noqa: E402
import ws2812  # noqa: E402

PROJECT = "tt_um_sulems6_metastable"


async def board(dut, loopback=True, clocks_per_step=30):
    """Reset the design and put it behind the stand-in. With `loopback`
    the pads read back what the chip drives on them."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())  # 50 MHz
    dut.ena.value = 1
    dut.ui_in.value = 0x02  # CS_N high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    demoboard.connect(dut, clocks_per_step)
    if loopback:
        cocotb.start_soon(uio_loopback(dut))


def host():
    """What every example's main() does first. Runs in the example's thread."""
    tt = demoboard.DemoBoard.get()
    getattr(tt.shuttle, PROJECT).enable()
    return uart_loopback.make_host(tt)


def dedup(seq):
    out = []
    for s in seq:
        if not out or s != out[-1]:
            out.append(s)
    return out


@cocotb.test()
async def test_example_uart_loopback(dut):
    """uart_loopback.main(): four bytes go out of SM0 and come back from SM1."""
    await board(dut)
    got = await bridge(uart_loopback.main)()
    assert list(got) == [0xA5, 0x3C, 0x00, 0xFF], f"main() returned {got}"
    assert demoboard.enabled() == [PROJECT], f"enabled {demoboard.enabled()}"


@cocotb.test()
async def test_example_spi_modes(dut):
    """spi_modes.main(): the four modes one after the other, with no reset
    between them, so each has to set up everything that it needs."""
    await board(dut)
    for mode in range(4):
        m_rx, s_rx = await bridge(spi_modes.main)(mode)
        assert list(m_rx) == [0xEF, 0x40, 0xAA, 0x81], f"mode {mode}: master got {m_rx}"
        assert list(s_rx) == [0x9E, 0x01, 0x55, 0xC3], f"mode {mode}: slave got {s_rx}"


@cocotb.test()
async def test_example_manchester(dut):
    """manchester.main(): a frame goes out of SM0 and SM1 decodes it."""
    await board(dut)
    got = await bridge(manchester.main)()
    assert list(got) == [0x0F, 0x96, 0x55, 0xC3], f"main() returned {got}"


@cocotb.test()
async def test_example_ws2812(dut):
    """ws2812.main(): two LEDs. The decoder has to see one frame, whole:
    a host-paced gap between two bytes would cut it in two."""
    await board(dut)
    dec = WS2812Decoder(dut)
    cocotb.start_soon(dec.run())
    await bridge(ws2812.main)(((255, 0, 0), (0, 0, 255)))
    # 48 bits of 1.25 us are 3,000 clock cycles, and the decoder then
    # needs its latch gap
    await ClockCycles(dut.clk, 5000)

    assert not dec.bad_pulses, f"out-of-spec pulses (ns): {dec.bad_pulses}"
    assert dec.frames == [[0, 255, 0, 0, 0, 255]], f"frames on the wire: {dec.frames}"


@cocotb.test()
async def test_example_replay(dut):
    """capture_replay.replay(): 16 symbols on GPIO4-7, each for exactly
    2 ticks. A symbol that waits for the host would last longer."""
    await board(dut)
    seen = []  # (clock cycle, level) of every change on GPIO4-7

    async def watch():
        t, prev = 0, 0
        while True:
            await RisingEdge(dut.clk)
            t += 1
            try:
                level = ((int(dut.uio_out.value) & int(dut.uio_oe.value)) >> 4) & 0xF
            except ValueError:
                continue
            if level != prev:
                seen.append((t, level))
                prev = level

    cocotb.start_soon(watch())
    wave = [0x1, 0x3, 0x7, 0xF, 0xE, 0xC, 0x8, 0x0] * 2

    def run():
        capture_replay.replay(host(), wave, div=16)

    await bridge(run)()
    await ClockCycles(dut.clk, 800)

    assert [level for _, level in seen] == wave, f"levels on the pins: {seen}"
    gaps = sorted(set(b[0] - a[0] for a, b in zip(seen, seen[1:])))
    assert gaps == [32], f"symbol lengths in clock cycles: {gaps}, not 32 for each"


@cocotb.test()
async def test_example_capture(dut):
    """capture_replay.capture(): a 16-sample snapshot of a waveform that
    the test puts on GPIO4-7 from outside the chip."""
    await board(dut, loopback=False)
    cycle = [0x1, 0x3, 0x7, 0xF, 0xE, 0xC, 0x8, 0x0]

    async def stimulus():
        i = 0
        while True:
            dut.uio_in.value = cycle[i % len(cycle)] << 4
            i += 1
            await ClockCycles(dut.clk, 64)  # two samples at divider 16

    cocotb.start_soon(stimulus())

    def run():
        return capture_replay.capture(host(), div=16)

    samples = await bridge(run)()
    assert len(samples) == 16, f"{len(samples)} samples, not 16"
    got = dedup(samples)
    assert len(got) >= 7, f"too few symbols in the snapshot: {samples}"
    for a, b in zip(got, got[1:]):
        want = cycle[(cycle.index(a) + 1) % len(cycle)]
        assert b == want, f"{b:#x} follows {a:#x} in the snapshot {samples}"


@cocotb.test()
async def test_example_i2c_write(dut):
    """i2c_write.main(): an address and one byte, acknowledged."""
    await board(dut, loopback=False)
    slave = I2CSlave(dut)
    cocotb.start_soon(slave.run())
    acks = await bridge(i2c_write.main)(0x52, (0x57,))
    await ClockCycles(dut.clk, 2000)  # the STOP follows the acknowledge bits

    assert acks == 0, f"acknowledge bits {acks}"
    assert slave.bytes == [0xA4, 0x57], f"the slave got {slave.bytes}"
    assert slave.stopped, "no STOP condition seen"


@cocotb.test()
async def test_example_i2c_read(dut):
    """i2c_read.main(): two bytes from a target, then an address with no
    device, which returns None."""
    await board(dut, loopback=False)
    bus = I2CTarget(dut, addr7=0x52, read_data=[0xC4, 0x1D])
    cocotb.start_soon(bus.run())

    data = await bridge(i2c_read.main)(0x52, 2)
    assert list(data) == [0xC4, 0x1D], f"main() returned {data}"
    assert bus.master_acks == [True, False], f"acknowledged {bus.master_acks}"

    nobody = await bridge(i2c_read.main)(0x29, 1)
    await ClockCycles(dut.clk, 1500)
    assert nobody is None, f"main() returned {nobody} for an address with no device"
    assert len(bus.starts) == 2 and len(bus.stops) == 2, \
        f"{len(bus.starts)} START and {len(bus.stops)} STOP conditions"


@cocotb.test()
async def test_example_ps2_device(dut):
    """ps2_device.main(): two frames at 12.5 kHz, checked by a host model."""
    await board(dut, loopback=False)
    model = Ps2Host(dut)
    cocotb.start_soon(model.run())
    await bridge(ps2_device.main)((0x1C, 0xF0))

    assert not model.errors, f"framing errors: {model.errors}"
    assert model.bytes == [0x1C, 0xF0], f"the host model got {model.bytes}"
    freq = 1e9 / (sum(model.periods_ns[:10]) / 10)
    assert 10e3 <= freq <= 16.7e3, f"clock {freq} Hz out of band"


@cocotb.test()
async def test_example_ps2_device_slow_host(dut):
    """ps2_device.send() with a host that is slower than the frame.

    The clock is ten times faster than in main(), and the host ten times
    slower, so one SPI byte takes longer than half a frame. Every clock
    period inside a frame still has the same length: no frame waits for
    the host in its middle.
    """
    await board(dut, loopback=False)
    model = Ps2Host(dut)
    cocotb.start_soon(model.run())
    codes = [0x1C, 0xF0, 0x5A]

    def run():
        h = host()
        ps2_device.setup(h, clkdiv=25)
        demoboard.set_speed(300)
        ps2_device.send(h, codes)

    await bridge(run)()
    assert not model.errors, f"framing errors: {model.errors}"
    assert model.bytes == codes, f"the host model got {model.bytes}"
    bit = 16 * 25 * 20  # ns: 16 ticks of 25 clock cycles
    inside = [p for p in model.periods_ns if p < 4 * bit]
    assert len(inside) == 10 * len(codes), f"{len(inside)} clock periods inside the frames"
    assert set(inside) == {bit}, f"clock periods inside a frame, ns: {sorted(set(inside))}"


@cocotb.test()
async def test_example_nec_ir_receive(dut):
    """nec_ir_receive.receive(): one frame from a remote control model.
    The NEC timing and the divider are both scaled by 20, as in test.py."""
    await board(dut, loopback=False)
    demoboard.bench_pins(1 << 3)  # the receiver's line idles high
    frame = [0x00, 0xFF, 0x2F, 0xD0]
    unit = 28125  # ns: 562.5 us / 20

    async def remote():
        # Wait until SM0 runs and stalls at its first WAIT.
        while str(dut.uo_out.value)[-6] != "1":
            await RisingEdge(dut.clk)
        await Timer(20 * unit, unit="ns")
        pulses = [(16, 8)]  # leader
        for b in frame:
            pulses += [(1, 3 if (b >> i) & 1 else 1) for i in range(8)]  # LSB first
        pulses.append((1, 4))  # stop burst
        for low, high in pulses:
            demoboard.bench_pins(0)
            await Timer(low * unit, unit="ns")
            demoboard.bench_pins(1 << 3)
            await Timer(high * unit, unit="ns")

    cocotb.start_soon(remote())

    def run():
        h = host()
        nec_ir_receive.setup(h, clkdiv=100)
        return nec_ir_receive.receive(h)

    got = await bridge(run)()
    assert got == (0x00, 0x2F), f"receive() returned {got}"
