#!/usr/bin/env python3
# Metastable protocol emulator - RTL against the post-layout netlist
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
"""Prove that the post-layout netlist computes what the RTL computes.

The gate-level simulation runs the netlist on the tests that someone
wrote. This script makes a statement about every input and every state.

What it proves
--------------
Take any clock cycle. Suppose that every flip-flop of the netlist holds
the value of its partner in the RTL, and that both designs get the same
inputs. Then:

  1. every output pin has the same value in the netlist and in the RTL;
  2. after the next clock edge, every flip-flop of the netlist again
     holds the value of its partner.

By induction the two designs then agree in every later cycle. Every
flip-flop of the netlist has a partner, so the netlist has no state that
the proof leaves out.

How a flip-flop gets its partner
--------------------------------
  register   The net at the flip-flop's output has the name of a wire in
             the RTL. Synthesis keeps those names.
  port       The flip-flop drives an output pin. Its partner is the
             pin's driver in the RTL.
  duplicate  The net has a tool-made name. Synthesis copies the read
             address of a memory into flip-flops of its own. The script
             finds the named flip-flop with the same data input: it
             proves, in the netlist alone, that the two data inputs are
             the same function of the design's state and inputs. The
             partner in the RTL is that flip-flop's partner.

A register bit of the RTL that never changes has no flip-flop in the
netlist: synthesis replaced it by its value. CONSTANTS lists such bits.
For each one the script proves two things in the RTL: a clock edge with
reset active gives the bit that value, and the bit keeps the value once
it has it.

A register bit of the RTL that drives nothing has no flip-flop either.
The proofs leave such a bit free, so they also show that no output and
no flip-flop depends on it.

How the pairs come to agree in the first place
----------------------------------------------
After one clock edge each duplicate equals its named flip-flop, whatever
the state was before: that is what the duplicate proof says. A second
clock edge with reset active then gives every register that has a reset
value that value in both designs. A register with no reset value (the
instruction memory, the FIFO storage) starts with whatever the chip
powers up with, and the RTL is taken to start with the same contents.

What it does not prove
----------------------
  - Anything about time: setup, hold, clock skew. The timing reports
    cover those.
  - That the function the cell library states for a cell is what the
    cell's layout does.
  - Where Yosys leaves a value of the RTL open (x), the proof reads 0.

Usage
-----
  formal/netlist_equiv.py --netlist NETLIST --liberty LIBERTY
  formal/netlist_equiv.py --netlist NETLIST --liberty LIBERTY --control

NETLIST is the flat netlist without power pins,
runs/wokwi/final/nl/tt_um_sulems6_metastable.nl.v. A local run writes
it, and the GDS_logs artifact of the gds workflow holds it. LIBERTY is
sg13cmos5l_stdcell_typ_1p20V_25C.lib of the kit.

--control changes one cell of the netlist to a different gate and runs
the same proof. The proof has to fail: a check that cannot fail proves
nothing. The exit status is 0 when the proof holds (without --control)
or fails (with it). Tested with Yosys 0.68.
"""
import argparse
import collections
import hashlib
import json
import os
import re
import subprocess
import sys
import time

TOP = "tt_um_sulems6_metastable"
SOURCES = ["project.v", "spi_slave.v", "pio_sm.v", "pio_fifo.v", "pio_clkdiv.v"]

# Register bits of the RTL that hold one value for ever, and why.
CONSTANTS = {
    # Every assignment to shreg_out puts 0 in bit 0: the reset, the load
    # {rdata[6:0], 1'b0} and the shift {shreg_out[6:0], 1'b0}.
    "u_spi.shreg_out[0]": 0,
}

FLOP_PINS = {"D", "Q", "CLK", "RESET_B"}   # the one kind of flip-flop this script knows
KEYWORDS = {"module", "input", "output", "inout", "wire", "assign", "endmodule"}
ANCHORS = 3                                # pairs at each end of a list that show its order


def die(msg):
    print("netlist_equiv: " + msg, file=sys.stderr)
    sys.exit(2)


def net(text):
    """A net of the netlist as Yosys names it: no escape, no blanks."""
    return text.strip().lstrip("\\").strip()


def group(name):
    return re.sub(r"\[\d+\]", "[]", name)


class Netlist:
    """The cells, the flip-flops and the ports of a flat Verilog netlist."""

    def __init__(self, path, top):
        self.text = open(path).read()
        if not re.search(r"^\s*module\s+%s\b" % re.escape(top), self.text, re.M):
            die("%s has no module %s" % (path, top))
        if len(re.findall(r"^\s*module\s", self.text, re.M)) != 1:
            die("%s has more than one module: give the flat netlist" % path)
        self.inputs, self.outputs = [], []           # (name, width)
        for kind, hi, lo, name in re.findall(
                r"^\s*(input|output)\s*(?:\[(\d+):(\d+)\])?\s*([^\s;]+)\s*;", self.text, re.M):
            if hi and int(lo) != 0:
                die("port %s does not start at bit 0" % name)
            width = int(hi) + 1 if hi else 1
            (self.inputs if kind == "input" else self.outputs).append((net(name), width))
        if not self.inputs or not self.outputs:
            die("%s: no ports found" % path)
        self.cells = collections.OrderedDict()       # instance -> (cell type, {pin: net})
        for ctype, name, body in re.findall(
                r"^\s*(\w+)\s+(\\\S+\s|[^\s(]+)\s*\((.*?)\);", self.text, re.S | re.M):
            if ctype in KEYWORDS:
                continue
            pins = {p: net(n) for p, n in re.findall(r"\.(\w+)\(\s*(.*?)\s*\)", body, re.S)}
            self.cells[net(name)] = (ctype, pins)
        self.flops = collections.OrderedDict()       # instance -> {pin: net}
        for name, (ctype, pins) in self.cells.items():
            if "Q" in pins and "D" in pins and "CLK" in pins:
                if set(pins) - FLOP_PINS:
                    die("flip-flop %s (%s) has pins that this script does not know: %s"
                        % (name, ctype, sorted(set(pins) - FLOP_PINS)))
                self.flops[name] = pins
        self.by_q = {pins["Q"]: name for name, pins in self.flops.items()}
        if len(self.by_q) != len(self.flops):
            die("two flip-flops drive one net")

    def output_bits(self):
        return {("%s[%d]" % (n, i)) if w > 1 else n for n, w in self.outputs for i in range(w)}

    def quiet_input(self):
        """One input bit to build the constant wires from: `ena` if the
        design has it (Tiny Tapeout designs do, and do not use it)."""
        for n, w in self.inputs:
            if n == "ena":
                return n
        n, w = self.inputs[-1]
        return n + ("[0]" if w > 1 else "")


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def link(work, name, target):
    """Yosys reads its inputs through links in the work directory, so no
    path in a script can hold a blank."""
    path = os.path.join(work, name)
    if os.path.islink(path) or os.path.exists(path):
        os.remove(path)
    os.symlink(os.path.abspath(target), path)


def yosys(work, tag, script):
    open(os.path.join(work, tag + ".ys"), "w").write(script)
    t0 = time.time()
    r = subprocess.run(["yosys", "-q", "-l", tag + ".log", tag + ".ys"], cwd=work,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    if r.returncode != 0:
        die("yosys failed on %s.ys in %s:\n%s" % (tag, work, r.stdout[-2000:]))
    return time.time() - t0


def equiv_cells(path):
    """{bit of the $equiv output: (A, B)} from a Yosys dump."""
    out = {}
    for block in re.findall(r"cell \$equiv .*?\n(.*?)\n\s*end", open(path).read(), re.S):
        conn = dict(re.findall(r"connect \\(\w) (.*)", block))
        m = re.search(r"\[(\d+)\]\s*$", conn["Y"])
        out[int(m.group(1)) if m else 0] = (conn["A"].strip(), conn["B"].strip())
    return out


def anchored(pairs):
    """Yosys numbers the bits of a list of signals from one end. A wire
    paired with itself at the head of the list, and two different wires
    at its tail, show from which end."""
    return [("zero", "zero")] * ANCHORS + list(pairs) + [("zero", "also_zero")] * ANCHORS


def results(work, made, done, n):
    """For each of the n pairs between the anchors: is it proven."""
    made = equiv_cells(os.path.join(work, made))
    done = equiv_cells(os.path.join(work, done))
    total = n + 2 * ANCHORS
    if len(made) != total or len(done) != total:
        die("%d pairs written, %d and %d cells found" % (total, len(made), len(done)))
    same = lambda cells, idx: [cells[i][0] == cells[i][1] for i in idx]
    low, high = range(ANCHORS), range(total - ANCHORS, total)
    if all(same(made, low)) and not any(same(made, high)):
        index = lambda i: i
    elif all(same(made, high)) and not any(same(made, low)):
        index = lambda i: total - 1 - i
    else:
        die("cannot tell the order of the pairs in the Yosys dump")
    if not all(same(done, low)) or not all(same(done, high)):
        die("an anchor pair is not proven: the proof set-up is broken")
    return [done[index(ANCHORS + i)][0] == done[index(ANCHORS + i)][1] for i in range(n)]


READ_RTL = """read_verilog %s
hierarchy -check -top %s
proc
memory_map
flatten
"""

READ_NETLIST = """read_liberty -ignore_miss_func cells.lib
read_verilog netlist.v
hierarchy -check -top %s
proc
flatten
rename %s gate
"""


def wrapper(nl, name, with_gold):
    """A module that holds the netlist (h) and, if asked, the RTL (g) on
    the same inputs. `zero` and `also_zero` are two wires that are
    always 0, each the output of its own cell."""
    vec = lambda w: "[%d:0] " % (w - 1) if w > 1 else ""
    sides = [("gate", "h")] + ([("gold", "g")] if with_gold else [])
    ports = ["input %s%s" % (vec(w), n) for n, w in nl.inputs]
    for side, _ in sides:
        ports += ["output %s%s_%s" % (vec(w), n, side) for n, w in nl.outputs]
    q = nl.quiet_input()
    body = ["  wire zero = %s & ~%s;" % (q, q), "  wire also_zero = ~(%s | ~%s);" % (q, q)]
    for side, inst in sides:
        conns = [".%s(%s)" % (n, n) for n, _ in nl.inputs]
        conns += [".%s(%s_%s)" % (n, n, side) for n, _ in nl.outputs]
        body.append("  %s %s(%s);" % (side, inst, ", ".join(conns)))
    return "module %s(%s);\n%s\nendmodule\n" % (name, ", ".join(ports), "\n".join(body))


def rtl_names(work, top, sources):
    """Every wire name of the flat RTL, bit by bit, and the names of
    each of its register bits. opt_expr folds the read of a register
    array at a fixed index into a plain connection, so that such a
    register bit also carries the name of the wire that reads it."""
    yosys(work, "rtl", READ_RTL % (sources, top) + "opt_expr\nsplitnets\nwrite_json rtl.json\n")
    mod = json.load(open(os.path.join(work, "rtl.json")))["modules"][top]
    names = collections.defaultdict(set)             # bit -> names
    for name, item in mod["netnames"].items():
        for i, bit in enumerate(item["bits"]):
            if isinstance(bit, int):
                names[bit].add(name if len(item["bits"]) == 1 else "%s[%d]" % (name, i))
    regs = []
    for cell in mod["cells"].values():
        if re.fullmatch(r"\$(a|s|al)?dff(sr)?(c?e)?|\$ff", cell["type"]):
            regs += [names[b] for b in cell["connections"]["Q"] if isinstance(b, int)]
    wires = set()
    for item in names.values():
        wires |= item
    return wires, regs


def find_twins(work, nl, top, unnamed, named):
    """For each flip-flop with a tool-made name, the named flip-flops
    that have the same data input. Proved in the netlist alone, with
    every flip-flop output free."""
    d_of = lambda q: "h." + nl.flops[nl.by_q[q]]["D"]
    pairs = [(d_of(c), d_of(d)) for d in unnamed for c in named]
    full = anchored(pairs)
    open(os.path.join(work, "twin_wrapper.v"), "w").write(wrapper(nl, "twin", False))
    secs = yosys(work, "twin", READ_NETLIST % (top, top) + """read_verilog twin_wrapper.v
hierarchy -check -top twin
flatten
async2sync
cd twin
equiv_add %s %s
cd ..
dump -o twin_made.txt t:$equiv
equiv_simple -seq 0
dump -o twin_done.txt t:$equiv
""" % (",".join(a for a, _ in full), ",".join(b for _, b in full)))
    proven = iter(results(work, "twin_made.txt", "twin_done.txt", len(pairs)))
    found = collections.OrderedDict((d, []) for d in unnamed)
    for d in unnamed:
        for c in named:
            if next(proven):
                found[d].append(c)
    return found, secs


def base_case(work, top, sources, reset, constants):
    """A clock edge with reset active gives each constant bit its value."""
    name, level = reset
    proves = "".join(" -prove %s 1'b%d" % (bit, value) for bit, value in sorted(constants.items()))
    return yosys(work, "base", READ_RTL % (sources, top) +
                 "sat -verify -seq 2 -prove-skip 1 -set-at 1 %s 1'b%d%s\n" % (name, level, proves))


def control_netlist(nl, work):
    """A copy of the netlist with one two-input gate changed to its
    opposite: the first NAND or NOR that drives an output pin, or else
    the first one in the file."""
    swap = {"nand2": "nor2", "nor2": "nand2"}
    outs = nl.output_bits()
    kind = lambda ctype: re.search(r"_(nand2|nor2)_", ctype)
    picks = [n for n, (t, p) in nl.cells.items() if kind(t) and p.get("Y") in outs]
    picks = picks or [n for n, (t, p) in nl.cells.items() if kind(t)]
    if not picks:
        die("no two-input NAND or NOR in the netlist to change")
    name = picks[0]
    old = nl.cells[name][0]
    new = old.replace(kind(old).group(1), swap[kind(old).group(1)])
    text, count = re.subn(r"(^\s*)%s(\s+%s\s*\()" % (re.escape(old), re.escape(name)),
                          r"\g<1>%s\g<2>" % new, nl.text, count=1, flags=re.M)
    if count != 1:
        die("could not change cell %s" % name)
    path = os.path.join(work, "control_netlist.v")
    open(path, "w").write(text)
    return path, {"cell": name, "was": old, "now": new, "drives": nl.cells[name][1].get("Y")}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--netlist", required=True, help="the flat post-layout netlist, without power pins")
    ap.add_argument("--liberty", required=True, help="the cell library's Liberty file")
    ap.add_argument("--src", default=os.path.join(here, "..", "src"), help="the directory of the RTL")
    ap.add_argument("--top", default=TOP)
    ap.add_argument("--work", default=os.path.join(here, "out", "netlist_equiv"),
                    help="where the Yosys scripts, logs and dumps go")
    ap.add_argument("--reset", default="rst_n=0", metavar="NAME=LEVEL",
                    help="the reset input and its active level")
    ap.add_argument("--control", action="store_true", help="change one cell and expect the proof to fail")
    ap.add_argument("--report", help="write the result to this JSON file")
    args = ap.parse_args()

    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    reset = (args.reset.split("=")[0], int(args.reset.split("=")[1]))
    for f in SOURCES:
        link(work, "rtl_" + f, os.path.join(args.src, f))
    sources = " ".join("rtl_" + f for f in SOURCES)
    link(work, "cells.lib", args.liberty)

    nl = Netlist(args.netlist, args.top)
    changed = None
    if args.control:
        path, changed = control_netlist(nl, work)
        print("control: cell %(cell)s changed from %(was)s to %(now)s (it drives %(drives)s)" % changed)
        nl = Netlist(path, args.top)
        link(work, "netlist.v", path)
    else:
        link(work, "netlist.v", args.netlist)

    wires, regs = rtl_names(work, args.top, sources)
    out_bits = nl.output_bits()
    port = sorted(q for q in nl.by_q if q in out_bits)
    named = sorted(q for q in nl.by_q if q not in out_bits and q in wires)
    unnamed = sorted(q for q in nl.by_q if q not in out_bits and q not in wires)
    print("netlist: %d flip-flops = %d with the name of an RTL wire + %d on an output pin + %d with a tool-made name"
          % (len(nl.flops), len(named), len(port), len(unnamed)))

    twins, twin_secs, no_twin = collections.OrderedDict(), 0.0, []
    if unnamed:
        found, twin_secs = find_twins(work, nl, args.top, unnamed, named + port)
        for d in unnamed:
            if found[d]:
                twins[d] = found[d][0]
            else:
                no_twin.append(d)
        print("duplicates: %d of %d have a named flip-flop with the same data input (%.0f s)"
              % (len(twins), len(unnamed), twin_secs))
    if no_twin and not args.control:
        die("flip-flops with no partner: %s" % ", ".join(no_twin))

    for bit, value in CONSTANTS.items():
        if bit not in wires:
            die("CONSTANTS names %s, and the RTL has no such wire" % bit)
        if value != 0:
            die("CONSTANTS: only the value 0 is supported")
    base_secs = base_case(work, args.top, sources, reset, CONSTANTS) if CONSTANTS else 0.0
    if CONSTANTS:
        print("constants: a clock edge in reset gives %d register bit(s) the stated value (%.0f s)"
              % (len(CONSTANTS), base_secs))

    # The pairs: (kind, RTL signal, netlist signal).
    pairs = []
    for n, w in nl.outputs:
        for i in range(w):
            bit = "[%d]" % i if w > 1 else ""
            pairs.append(("output", "%s_gold%s" % (n, bit), "%s_gate%s" % (n, bit)))
    pairs += [("register", "g." + q, "h." + q) for q in named]
    pairs += [("duplicate", "g." + c, "h." + d) for d, c in twins.items()]
    pairs += [("constant", "g." + bit, "zero") for bit in sorted(CONSTANTS)]
    for _, a, b in pairs:
        if "," in a + b or " " in a + b:
            die("a signal name has a comma or a blank: %s %s" % (a, b))
    covered = len(port) + len(named) + len(twins)
    if covered != len(nl.flops) and not args.control:
        die("only %d of %d flip-flops are in a pair" % (covered, len(nl.flops)))

    full = anchored((a, b) for _, a, b in pairs)
    open(os.path.join(work, "wrapper.v"), "w").write(wrapper(nl, "equiv", True))
    secs = yosys(work, "proof", READ_RTL % (sources, args.top) + """splitnets
rename %s gold
design -stash gold
""" % args.top + READ_NETLIST % (args.top, args.top) + """design -copy-from gold gold
read_verilog wrapper.v
hierarchy -check -top equiv
flatten
cd equiv
equiv_add %s %s
cd ..
async2sync
opt_clean
dump -o proof_made.txt t:$equiv
equiv_simple -seq 1 -nogroup
dump -o proof_step.txt t:$equiv
equiv_induct
dump -o proof_done.txt t:$equiv
""" % (",".join(a for a, _ in full), ",".join(b for _, b in full)))
    one_step = results(work, "proof_made.txt", "proof_step.txt", len(pairs))
    final = results(work, "proof_made.txt", "proof_done.txt", len(pairs))

    table = collections.OrderedDict()
    unproven = []
    for (kind, a, b), s, f in zip(pairs, one_step, final):
        row = table.setdefault(kind, collections.OrderedDict(
            [("pairs", 0), ("proven_in_one_step", 0), ("proven_by_induction", 0), ("not_proven", 0)]))
        row["pairs"] += 1
        row["proven_in_one_step"] += int(s)
        row["proven_by_induction"] += int(f and not s)
        row["not_proven"] += int(not f)
        if not f:
            unproven.append({"kind": kind, "rtl": a[2:] if a.startswith("g.") else a,
                             "netlist": b[2:] if b.startswith("h.") else b})

    # Register bits of the RTL with no partner. A bit has several names;
    # it has a partner if one of them is in a pair or is an output pin.
    in_pair = {a[2:] for _, a, _ in pairs if a.startswith("g.")} | out_bits
    lonely, temporaries = collections.Counter(), 0
    for names_of_bit in regs:
        public = sorted(n for n in names_of_bit if not n.startswith("$"))
        if not public:
            temporaries += 1
        elif not in_pair.intersection(public):
            lonely[group(public[0])] += 1

    total = sum(r["pairs"] for r in table.values())
    bad = sum(r["not_proven"] for r in table.values())
    print("proof: %d pairs, %d proven, %d not proven (%.0f s)" % (total, total - bad, bad, secs))
    for kind, row in table.items():
        print("  %-9s %5d pairs  %5d in one step  %3d by induction  %3d not proven"
              % (kind, row["pairs"], row["proven_in_one_step"], row["proven_by_induction"], row["not_proven"]))
    for item in unproven[:20]:
        print("  not proven: %(kind)s  RTL %(rtl)s  netlist %(netlist)s" % item)
    print("RTL register bits with no flip-flop in the netlist: %d %s; with a tool-made name: %d"
          % (sum(lonely.values()), dict(sorted(lonely.items())), temporaries))

    proved = bad == 0 and not no_twin
    if args.control:
        verdict = "refuted, as a control has to be" if not proved else "PROVED, and a control must not be"
    else:
        verdict = "PROVED" if proved else "NOT PROVED"
    print("netlist against RTL: " + verdict)
    if args.report:
        tool = subprocess.run(["yosys", "-V"], stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
        report = collections.OrderedDict([
            ("netlist", os.path.basename(args.netlist)),
            ("sha256", collections.OrderedDict(
                [("netlist", sha256(args.netlist)), ("liberty", sha256(args.liberty))] +
                [(f, sha256(os.path.join(args.src, f))) for f in SOURCES])),
            ("yosys", tool),
            ("control", changed),
            ("flip_flops", collections.OrderedDict([
                ("total", len(nl.flops)), ("with_the_name_of_an_rtl_wire", len(named)),
                ("on_an_output_pin", len(port)), ("with_a_tool_made_name", len(unnamed))])),
            ("duplicates", twins),
            ("duplicates_with_no_partner", no_twin),
            ("constants", CONSTANTS),
            ("pairs", table),
            ("not_proven", unproven),
            ("rtl_register_bits_with_no_flip_flop", dict(sorted(lonely.items()))),
            ("rtl_register_bits_with_a_tool_made_name", temporaries),
            ("seconds", {"duplicates": round(twin_secs), "constants": round(base_secs), "proof": round(secs)}),
            ("verdict", verdict),
        ])
        with open(args.report, "w") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
    return int(proved) if args.control else int(not proved)


if __name__ == "__main__":
    sys.exit(main())
