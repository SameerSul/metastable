#!/usr/bin/env python3
# Metastable protocol emulator - coverage table of the constrained-random co-simulation
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
"""Print which instruction forms the constrained-random tests executed.

    python3 test/cosim_coverage.py              print the table
    python3 test/cosim_coverage.py --check      also exit 1 on an empty cell

The two constrained-random tests of test.py each write a file when all of
their seeds agree with the hardware: cosim_coverage_arch.json and
cosim_coverage_pins.json. A cell of the table counts how often the
golden model (golden.py) executed one form of an instruction in those
runs. golden.COVERAGE_ROWS lists the forms: every operand value of the
documented instruction set, and the outcomes of the instructions that
can stall.

A push runs 12 + 8 seeds, which leaves some cells empty. The scheduled
job runs 120 + 80 seeds and uses --check.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sw"))
sys.path.insert(0, HERE)

from golden import COVERAGE_ROWS  # noqa: E402

FILES = ["cosim_coverage_arch.json", "cosim_coverage_pins.json"]


def load(directory):
    runs = []
    for name in FILES:
        path = os.path.join(directory, name)
        if not os.path.exists(path):
            sys.exit("cosim_coverage: %s is missing: run both constrained-random tests first" % path)
        with open(path) as f:
            runs.append(json.load(f))
    return runs


def table(runs):
    """The rows of the table and the names of its empty cells."""
    cells = {}
    for run in runs:
        for key, count in run["cells"].items():
            cells[key] = cells.get(key, 0) + count
    known = {"%s: %s" % (row, value) for row, values in COVERAGE_ROWS for value in values}
    stray = sorted(set(cells) - known)
    if stray:
        sys.exit("cosim_coverage: the runs hold cells that the table does not list: %s" % stray)
    rows, empty = [], []
    for row, values in COVERAGE_ROWS:
        counts = [(value, cells.get("%s: %s" % (row, value), 0)) for value in values]
        rows.append((row, counts))
        empty += ["%s: %s" % (row, value) for value, count in counts if count == 0]
    return rows, empty


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", default=HERE, help="where the two result files are")
    ap.add_argument("--check", action="store_true", help="exit 1 when a cell is empty")
    args = ap.parse_args()

    runs = load(args.dir)
    rows, empty = table(runs)
    seeds = sum(run["seeds"] for run in runs)
    print("Constrained-random co-simulation: %d seeds (%d architectural, %d on the pins), "
          "%d instructions executed before the park loop, %d seeds drawn again because "
          "their program did not end."
          % (seeds, runs[0]["seeds"], runs[1]["seeds"],
             sum(run["steps"] for run in runs), sum(run["resampled"] for run in runs)))
    print()
    print("| Instruction form | Times executed |")
    print("| --- | --- |")
    for row, counts in rows:
        print("| %s | %s |" % (row, ", ".join("%s: %d" % (value, count) for value, count in counts)))
    print()
    total = sum(len(counts) for _, counts in rows)
    print("%d of %d cells are filled." % (total - len(empty), total))
    for name in empty:
        print("empty: " + name)
    return 1 if (args.check and empty) else 0


if __name__ == "__main__":
    sys.exit(main())
