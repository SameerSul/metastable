# Metastable protocol emulator - a stand-in for the demo board's library
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
"""A stand-in for ttboard.demoboard, with the simulated design behind it.

The examples in sw/examples are written for the Tiny Tapeout demo board.
Each one imports DemoBoard, enables the project and moves three pins.
This module has the same names. Behind them is the design in the
simulator, so the suite runs each example's own host code: its main(),
the driver of sw/metastable_host.py, and the program, together.

A test calls connect() before the example runs. The example then runs in
a thread of its own (cocotb.task.bridge), and every pin write of the host
lets a number of clock cycles pass. That number makes the host slow, as a
bit-banged host is: an example that only works with a fast host fails.

The host owns ui[2:0], the SPI pins. A test that drives ui[7:3] calls
bench_pins(), so that the two never overwrite each other.
"""
from cocotb.task import resume
from cocotb.triggers import ClockCycles

_state = {"dut": None, "step": 30, "host": 0x02, "bench": 0x00, "enabled": [], "writes": 0}


def connect(dut, clocks_per_step=30):
    """Put the simulated design behind the stand-in."""
    _state.update(dut=dut, step=clocks_per_step, host=0x02, bench=0x00,
                  enabled=[], writes=0)


def set_speed(clocks_per_step):
    """How many clock cycles pass for each pin write of the host."""
    _state["step"] = clocks_per_step


def enabled():
    """The projects that an example enabled, by name."""
    return list(_state["enabled"])


def pin_writes():
    """How many times the host has written its pins."""
    return _state["writes"]


def bench_pins(value):
    """Set ui[7:3] from a test. Call it from a coroutine of the test."""
    _state["bench"] = value & 0xF8
    _state["dut"].ui_in.value = _state["bench"] | _state["host"]


@resume
async def _host_pins(value):
    _state["host"] = value & 0x07
    _state["writes"] += 1
    _state["dut"].ui_in.value = _state["bench"] | _state["host"]
    await ClockCycles(_state["dut"].clk, _state["step"])


@resume
async def _outputs():
    bits = str(_state["dut"].uo_out.value)
    return int("".join(b if b in "01" else "0" for b in bits), 2)


class _HostPins:
    """tt.ui_in: the pins that the host drives."""

    @property
    def value(self):
        return _state["bench"] | _state["host"]

    @value.setter
    def value(self, value):
        _host_pins(value)


class _ChipPins:
    """tt.uo_out: the pins that the chip drives."""

    @property
    def value(self):
        return _outputs()


class _Project:
    def __init__(self, name):
        self.name = name

    def enable(self):
        _state["enabled"].append(self.name)


class _Shuttle:
    def __getattr__(self, name):
        return _Project(name)


class DemoBoard:
    """The part of the demo board's DemoBoard that the examples use."""

    _board = None

    def __init__(self):
        self.shuttle = _Shuttle()
        self.ui_in = _HostPins()
        self.uo_out = _ChipPins()

    @classmethod
    def get(cls):
        if cls._board is None:
            cls._board = cls()
        return cls._board
