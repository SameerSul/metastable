#!/usr/bin/env python3
# Metastable protocol emulator - the README's test count, from the result file
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
"""Keep the test count in README.md equal to the suite's result file.

    python3 test/readme_counts.py --check    exit 1 when the README is stale
    python3 test/readme_counts.py --write    rewrite the count in the README

The count is the number of test cases in test/results.xml, the file that
a run of the cocotb suite writes. A result file with a failed or skipped
test is refused, because the README states a count of passing tests.
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
README = os.path.join(HERE, "..", "README.md")
RESULTS = os.path.join(HERE, "results.xml")
COUNT = re.compile(r"(\| Verification \| )(\d+)( cocotb tests)")


def results(path):
    """(test cases, cases that did not pass) of a cocotb result file."""
    total = bad = 0
    for case in ET.parse(path).getroot().iter("testcase"):
        total += 1
        if any(child.tag in ("failure", "error", "skipped") for child in case):
            bad += 1
    return total, bad


def stated(text):
    """The count that the README states, or None."""
    found = COUNT.findall(text)
    return int(found[0][1]) if len(found) == 1 else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="exit 1 when the README is stale")
    mode.add_argument("--write", action="store_true", help="rewrite the count in the README")
    ap.add_argument("--results", default=RESULTS, help="the cocotb result file")
    ap.add_argument("--readme", default=README, help="the README to check or rewrite")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.results):
        print("error: no result file at %s: run the suite first" % args.results, file=sys.stderr)
        return 2
    total, bad = results(args.results)
    if total == 0 or bad:
        print("error: %d of %d tests did not pass, so there is no count to state" % (bad, total),
              file=sys.stderr)
        return 1
    with open(args.readme, encoding="utf-8") as f:
        text = f.read()
    have = stated(text)
    if have is None:
        print("error: the README has no line that states the test count", file=sys.stderr)
        return 2
    if have == total:
        print("the README states %d tests, and %d passed" % (have, total))
        return 0
    if args.check:
        print("error: the README states %d tests, and %d passed: run "
              "python3 test/readme_counts.py --write" % (have, total), file=sys.stderr)
        return 1
    with open(args.readme, "w", encoding="utf-8") as f:
        f.write(COUNT.sub(lambda m: m.group(1) + str(total) + m.group(3), text))
    print("the README now states %d tests, where it stated %d" % (total, have))
    return 0


if __name__ == "__main__":
    sys.exit(main())
