# Metastable protocol emulator - golden architectural model + program generator
# Copyright (c) 2026 Sameer Suleman
# SPDX-License-Identifier: Apache-2.0
#
# A cycle-free architectural model of pio_sm for constrained-random
# equivalence checking: given the same program, configuration and TX FIFO
# preload, it predicts PC, X, Y, IRQ flags and the exact RX FIFO contents.
# Semantics mirror pio_sm.v line by line, including saturating shift
# counts, autopull/autopush refill rules, non-blocking fallbacks
# (PULL: OSR<=X; PUSH: data dropped), the unconditional decrement of
# JMP X--/Y--, and every stall that parks the machine.
#
# The model also covers WAIT, the computed jumps (OUT PC, MOV PC), the
# injected instruction of MOV EXEC and the IRQ that waits. It counts the
# ticks that the hardware needs (one for an instruction and one for each
# tick of its delay), and it records which instruction forms a run
# executed: COVERAGE_ROWS lists them, and cosim_coverage.py prints the
# table.

import collections
import random

from metastable_asm import (
    JMP, WAIT, IN_, OUT, PUSH, PULL, MOV, IRQ, SET, side,
    SRC_PINS, SRC_X, SRC_Y, SRC_NULL, SRC_STATUS, SRC_ISR, SRC_OSR,
    ODST_PINS, ODST_X, ODST_Y, ODST_NULL, ODST_PINDIRS, ODST_PC, ODST_ISR,
    MDST_PINS, MDST_X, MDST_Y, MDST_EXEC, MDST_PC, MDST_ISR, MDST_OSR,
    M_COPY, M_INV, M_REV,
    SDST_PINS, SDST_X, SDST_Y, SDST_PINDIRS,
    C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC, C_XNEY, C_PIN, C_NOTOSRE,
    W_GPIO, W_PIN, W_IRQ,
)

BODY_LEN = 16          # random instructions of a program
PARK = BODY_LEN + 4    # after them: a 4-instruction dump of X and Y, then the park loop
PROG_LEN = 32          # the whole instruction memory: a computed jump can land anywhere

_SRC = {SRC_PINS: "PINS", SRC_X: "X", SRC_Y: "Y", SRC_NULL: "NULL",
        SRC_STATUS: "STATUS", SRC_ISR: "ISR", SRC_OSR: "OSR"}
_ODST = {ODST_PINS: "PINS", ODST_X: "X", ODST_Y: "Y", ODST_NULL: "NULL",
         ODST_PINDIRS: "PINDIRS", ODST_PC: "PC", ODST_ISR: "ISR"}
_MDST = {MDST_PINS: "PINS", MDST_X: "X", MDST_Y: "Y", MDST_EXEC: "EXEC",
         MDST_PC: "PC", MDST_ISR: "ISR", MDST_OSR: "OSR"}
_SDST = {SDST_PINS: "PINS", SDST_X: "X", SDST_Y: "Y", SDST_PINDIRS: "PINDIRS"}
_COND = ["always", "!X", "X--", "!Y", "Y--", "X!=Y", "PIN", "!OSRE"]
_WSRC = {W_GPIO: "GPIO", W_PIN: "PIN", W_IRQ: "IRQ"}
_MOP = {M_COPY: "copy", M_INV: "invert", M_REV: "reverse"}

# The coverage table: each row names one operand or one outcome of an
# instruction, and lists every value that the documented instruction set
# gives it. A run fills a cell when the model executes that form and the
# hardware then ends in the state that the model predicts.
COVERAGE_ROWS = [
    ("JMP condition", _COND),
    ("JMP outcome", ["taken", "not taken"]),
    ("WAIT source and level", ["GPIO 0", "GPIO 1", "PIN 0", "PIN 1", "IRQ 0", "IRQ 1"]),
    ("WAIT outcome", ["passes", "parks"]),
    ("IN source", ["PINS", "X", "Y", "NULL", "ISR", "OSR"]),
    ("IN bit count", [str(n) for n in range(1, 9)]),
    ("IN autopush", ["none", "pushes", "parks on a full FIFO"]),
    ("OUT destination", ["PINS", "X", "Y", "NULL", "PINDIRS", "PC", "ISR"]),
    ("OUT bit count", [str(n) for n in range(1, 9)]),
    ("OUT autopull", ["none", "refills", "parks on an empty FIFO"]),
    ("PUSH flags", ["plain", "block", "iffull", "iffull block"]),
    ("PUSH outcome", ["pushes", "skips", "drops on a full FIFO", "parks on a full FIFO"]),
    ("PULL flags", ["plain", "block", "ifempty", "ifempty block"]),
    ("PULL outcome", ["pulls", "skips", "copies X on an empty FIFO", "parks on an empty FIFO"]),
    ("MOV destination", ["PINS", "X", "Y", "EXEC", "PC", "ISR", "OSR"]),
    ("MOV operation", ["copy", "invert", "reverse"]),
    ("MOV source", ["PINS", "X", "Y", "NULL", "STATUS", "ISR", "OSR"]),
    ("Injected instruction", ["jumps", "does not jump"]),
    ("IRQ action", ["set", "clear", "set and wait"]),
    ("IRQ flag", ["0", "1", "2", "3"]),
    ("SET destination", ["PINS", "X", "Y", "PINDIRS"]),
    ("Side-set", ["none", "always", "optional, on", "optional, off"]),
    ("Delay", ["0", "1 or more"]),
]


def _rev8(v):
    return int(f"{v & 0xFF:08b}"[::-1], 2)


def _pin_mask(base, cnt):
    m = 0
    for i in range(min(cnt, 8)):
        m |= 1 << ((base + i) % 16)
    return m


def _pin_spread(base, cnt, val):
    v = 0
    for i in range(min(cnt, 8)):
        if (val >> i) & 1:
            v |= 1 << ((base + i) % 16)
    return v


class GoldenSM:
    def __init__(self, program, shiftctrl, thresh, status_cfg, tx_fifo,
                 pin_cfg=None, pins=0, dirs=0):
        self.prog = program
        self.autopull = bool(shiftctrl & 1)
        self.autopush = bool(shiftctrl & 2)
        self.out_right = bool(shiftctrl & 4)
        self.in_right = bool(shiftctrl & 8)
        self.pull_eff = (thresh & 0xF) or 8
        self.push_eff = ((thresh >> 4) & 0xF) or 8
        self.status_cfg = status_cfg
        self.tx = list(tx_fifo)
        self.rx = []
        self.pc = 0
        self.x = self.y = self.osr = self.isr = 0
        self.osr_cnt = 8  # empty
        self.isr_cnt = 0
        self.irq = [0, 0, 0, 0]
        self.parked = False  # permanently stalled
        # pin model (single SM; GPIO0-7 looped back, 8-12 read 0,
        # 13-15 read back the output latch). Persists across restarts.
        self.pc_cfg = pin_cfg or {}
        self.pins = pins
        self.dirs = dirs
        self.exec_byte = None  # the instruction that MOV EXEC injected
        self.ticks = 0         # ticks that the hardware needs to get here
        self.cov = collections.Counter()

    def _hit(self, row, value):
        self.cov[row + ": " + str(value)] += 1

    def done(self, park=PARK):
        """Parked for good, or in the park loop with nothing injected."""
        return self.parked or (self.pc == park and self.exec_byte is None)

    # -- helpers matching the RTL scratch logic --------------------------
    def _osr_empty(self):
        return self.osr_cnt >= self.pull_eff

    def _status(self):
        level = len(self.rx) if self.status_cfg & 0x10 else len(self.tx)
        return 0xFF if level < (self.status_cfg & 0xF) else 0x00

    def _gpio_in(self):
        v = 0
        for p in range(8):        # bidirectional, pad loopback
            if (self.dirs >> p) & (self.pins >> p) & 1:
                v |= 1 << p
        for p in range(13, 16):   # out-only, internal readback
            if (self.pins >> p) & 1:
                v |= 1 << p
        return v                  # 8-12: input-only, undriven in cosim

    def _pins_in8(self):
        g = self._gpio_in()
        base = self.pc_cfg.get("in_base", 0)
        return sum(((g >> ((base + i) % 16)) & 1) << i for i in range(8))

    def _write_pins(self, base, cnt, val):
        m = _pin_mask(base, cnt)
        self.pins = (self.pins & ~m) | _pin_spread(base, cnt, val)

    def _write_dirs(self, base, cnt, val):
        m = _pin_mask(base, cnt)
        self.dirs = (self.dirs & ~m) | _pin_spread(base, cnt, val)

    def _side(self, ci):
        """(enabled, value) from the dss field, mirroring pio_sm."""
        cnt = self.pc_cfg.get("side_count", 0)
        if cnt == 0:
            return False, 0
        dss = (ci >> 8) & 0x1F
        if self.pc_cfg.get("side_opt", 0):
            return bool(dss & 0x10), (dss >> (4 - cnt)) & ((1 << cnt) - 1)
        return True, (dss >> (5 - cnt)) & ((1 << cnt) - 1)

    def _delay(self, ci):
        """The delay from the dss field: the bits that side-set leaves."""
        cnt = self.pc_cfg.get("side_count", 0)
        opt = 1 if (cnt and self.pc_cfg.get("side_opt", 0)) else 0
        return (ci >> 8) & ((1 << (5 - cnt - opt)) - 1)

    def _cover_dss(self, ci):
        cnt = self.pc_cfg.get("side_count", 0)
        en, _ = self._side(ci)
        if cnt == 0:
            self._hit("Side-set", "none")
        elif self.pc_cfg.get("side_opt", 0):
            self._hit("Side-set", "optional, on" if en else "optional, off")
        else:
            self._hit("Side-set", "always")
        self._hit("Delay", "1 or more" if self._delay(ci) else "0")

    def _apply_side(self, ci):
        en, val = self._side(ci)
        if en:
            self._write_pins(self.pc_cfg.get("side_base", 0),
                             self.pc_cfg.get("side_count", 0), val)

    def _src(self, code):
        return {SRC_PINS: self._pins_in8(), SRC_X: self.x, SRC_Y: self.y,
                SRC_NULL: 0, SRC_STATUS: self._status(), SRC_ISR: self.isr,
                SRC_OSR: self.osr}[code]

    def _park(self, ci):
        """A stall that nothing in a one-engine run ends. Side-set still
        applies on every attempt of a stalled instruction."""
        self._apply_side(ci)
        self._cover_dss(ci)
        self.parked = True
        return False

    def step(self):
        """Execute one instruction; returns False once parked."""
        if self.parked:
            return False
        # MOV EXEC injects {8'h00, value}: a JMP with no delay and no side
        # bits, which runs in place of the instruction at the PC.
        injected = self.exec_byte is not None
        ci = self.exec_byte if injected else self.prog[self.pc]
        self.exec_byte = None
        opc = ci >> 13
        n = 8 if (ci & 0x1F) == 0 or (ci & 0x1F) > 8 else ci & 0xF
        mask = 0xFF >> (8 - n)
        jump = None

        if opc == 0:  # JMP
            cond = (ci >> 5) & 7
            take = {C_ALWAYS: True, C_NOTX: self.x == 0,
                    C_XDEC: self.x != 0, C_NOTY: self.y == 0,
                    C_YDEC: self.y != 0, C_XNEY: self.x != self.y,
                    C_PIN: bool((self._gpio_in()
                                 >> self.pc_cfg.get("jmp_pin", 0)) & 1),
                    C_NOTOSRE: not self._osr_empty()}[cond]
            if cond == C_XDEC:
                self.x = (self.x - 1) & 0xFF
            if cond == C_YDEC:
                self.y = (self.y - 1) & 0xFF
            if take:
                jump = ci & 0x1F
            self._hit("JMP condition", _COND[cond])
            self._hit("JMP outcome", "taken" if take else "not taken")
            if injected:
                self._hit("Injected instruction", "jumps" if take else "does not jump")

        elif opc == 1:  # WAIT
            pol = (ci >> 7) & 1
            src = (ci >> 5) & 3
            idx = ci & 0xF

            def level():
                if src == W_GPIO:
                    return (self._gpio_in() >> idx) & 1
                if src == W_PIN:
                    return (self._gpio_in()
                            >> ((self.pc_cfg.get("in_base", 0) + idx) % 16)) & 1
                if src == W_IRQ:
                    return self.irq[idx & 3]
                return 1  # source 3 is not in the instruction set

            self._hit("WAIT source and level", "%s %d" % (_WSRC.get(src, "?"), pol))
            if level() != pol:
                # A stalled WAIT still drives its side-set pins, and one of
                # them can be the pin that it waits for.
                self._apply_side(ci)
                if level() != pol:
                    self._hit("WAIT outcome", "parks")
                    return self._park(ci)
            self._hit("WAIT outcome", "passes")
            if src == W_IRQ and pol:
                self.irq[idx & 3] = 0  # a wait for a set flag clears it

        elif opc == 2:  # IN
            src = (ci >> 5) & 7
            val = self._src(src) & mask
            if self.in_right:
                shifted = ((self.isr >> n) | (val << (8 - n))) & 0xFF
            else:
                shifted = ((self.isr << n) | val) & 0xFF
            cnt = min(self.isr_cnt + n, 8)
            self._hit("IN source", _SRC[src])
            self._hit("IN bit count", n)
            if self.autopush and cnt >= self.push_eff:
                if len(self.rx) == 8:
                    self._hit("IN autopush", "parks on a full FIFO")
                    return self._park(ci)  # stall on full RX
                self.rx.append(shifted)
                self.isr, self.isr_cnt = 0, 0
                self._hit("IN autopush", "pushes")
            else:
                self.isr, self.isr_cnt = shifted, cnt
                self._hit("IN autopush", "none")

        elif opc == 3:  # OUT
            dst = (ci >> 5) & 7
            refill = self.autopull and self._osr_empty()
            self._hit("OUT destination", _ODST.get(dst, "?"))
            self._hit("OUT bit count", n)
            if refill and not self.tx:
                self._hit("OUT autopull", "parks on an empty FIFO")
                return self._park(ci)
            self._hit("OUT autopull", "refills" if refill else "none")
            osr_eff = self.tx.pop(0) if refill else self.osr
            cnt_eff = 0 if refill else self.osr_cnt
            if self.out_right:
                bits = osr_eff & mask
                self.osr = osr_eff >> n
            else:
                bits = (osr_eff >> (8 - n)) & 0xFF
                self.osr = (osr_eff << n) & 0xFF
            self.osr_cnt = min(cnt_eff + n, 8)
            if dst == ODST_PINS:
                self._write_pins(self.pc_cfg.get("out_base", 0),
                                 self.pc_cfg.get("out_count", 0), bits)
            elif dst == ODST_X:
                self.x = bits
            elif dst == ODST_Y:
                self.y = bits
            elif dst == ODST_PINDIRS:
                self._write_dirs(self.pc_cfg.get("out_base", 0),
                                 self.pc_cfg.get("out_count", 0), bits)
            elif dst == ODST_PC:
                jump = bits & 0x1F
            elif dst == ODST_ISR:
                self.isr, self.isr_cnt = bits, n

        elif opc == 4:  # PUSH / PULL
            block = (ci >> 5) & 1
            iffull = (ci >> 6) & 1
            if (ci >> 7) & 1:  # PULL
                self._hit("PULL flags", (("ifempty " if iffull else "")
                                         + ("block" if block else "")).strip() or "plain")
                if not iffull or self._osr_empty():
                    if not self.tx:
                        if block:
                            self._hit("PULL outcome", "parks on an empty FIFO")
                            return self._park(ci)
                        self.osr, self.osr_cnt = self.x, 0
                        self._hit("PULL outcome", "copies X on an empty FIFO")
                    else:
                        self.osr, self.osr_cnt = self.tx.pop(0), 0
                        self._hit("PULL outcome", "pulls")
                else:
                    self._hit("PULL outcome", "skips")
            else:  # PUSH
                self._hit("PUSH flags", (("iffull " if iffull else "")
                                         + ("block" if block else "")).strip() or "plain")
                if not iffull or self.isr_cnt >= self.push_eff:
                    if len(self.rx) == 8:
                        if block:
                            self._hit("PUSH outcome", "parks on a full FIFO")
                            return self._park(ci)
                        self.isr, self.isr_cnt = 0, 0
                        self._hit("PUSH outcome", "drops on a full FIFO")
                    else:
                        self.rx.append(self.isr)
                        self.isr, self.isr_cnt = 0, 0
                        self._hit("PUSH outcome", "pushes")
                else:
                    self._hit("PUSH outcome", "skips")

        elif opc == 5:  # MOV
            val = self._src(ci & 7)
            op = (ci >> 3) & 3
            if op == M_INV:
                val = (~val) & 0xFF
            elif op == M_REV:
                val = _rev8(val)
            dst = (ci >> 5) & 7
            self._hit("MOV destination", _MDST.get(dst, "?"))
            self._hit("MOV operation", _MOP.get(op, "?"))
            self._hit("MOV source", _SRC[ci & 7])
            if dst == MDST_PINS:
                self._write_pins(self.pc_cfg.get("out_base", 0),
                                 self.pc_cfg.get("out_count", 0), val)
            elif dst == MDST_X:
                self.x = val
            elif dst == MDST_Y:
                self.y = val
            elif dst == MDST_EXEC:
                self.exec_byte = val  # {8'h00, val} runs next
            elif dst == MDST_PC:
                jump = val & 0x1F
            elif dst == MDST_ISR:
                self.isr, self.isr_cnt = val, 0
            elif dst == MDST_OSR:
                self.osr, self.osr_cnt = val, 0

        elif opc == 6:  # IRQ
            idx = ci & 3
            self._hit("IRQ flag", idx)
            if (ci >> 6) & 1:
                self.irq[idx] = 0
                self._hit("IRQ action", "clear")
            elif (ci >> 5) & 1:
                # set, then wait until the flag is clear again: nothing
                # clears it in a one-engine run, so the engine parks
                self.irq[idx] = 1
                self._hit("IRQ action", "set and wait")
                return self._park(ci)
            else:
                self.irq[idx] = 1
                self._hit("IRQ action", "set")

        elif opc == 7:  # SET
            dst = (ci >> 5) & 7
            self._hit("SET destination", _SDST.get(dst, "?"))
            if dst == SDST_PINS:
                self._write_pins(self.pc_cfg.get("set_base", 0),
                                 self.pc_cfg.get("set_count", 0), ci & 0x1F)
            elif dst == SDST_X:
                self.x = ci & 0x1F
            elif dst == SDST_Y:
                self.y = ci & 0x1F
            elif dst == SDST_PINDIRS:
                self._write_dirs(self.pc_cfg.get("set_base", 0),
                                 self.pc_cfg.get("set_count", 0), ci & 0x1F)

        self._apply_side(ci)  # side-set wins conflicts with the op above
        self._cover_dss(ci)
        self.ticks += 1 + self._delay(ci)
        if jump is not None:
            self.pc = jump
        elif not injected:  # an injected instruction leaves the PC alone
            self.pc = (self.pc + 1) & 0x1F
        return True


def _finish(body):
    """The whole instruction memory: the random body, a dump of X and Y
    into the RX FIFO, the park loop, and jumps to the park loop in every
    word that is left. A computed jump can therefore land anywhere."""
    dump = [MOV(MDST_ISR, SRC_X), PUSH(block=0),
            MOV(MDST_ISR, SRC_Y), PUSH(block=0)]
    prog = body + dump + [JMP(PARK)]
    assert len(prog) == PARK + 1
    return prog + [JMP(PARK)] * (PROG_LEN - len(prog))


def _with_delay(rng, ins, bits):
    """Now and then give an instruction a delay. `bits` is the width of
    the delay field that the side-set configuration leaves; the delay
    sits in the low bits of the field in every configuration."""
    if bits and rng.random() < 0.12:
        top = (1 << bits) - 1
        ins |= rng.choice([1, min(2, top), min(3, top), top]) << 8
    return ins


def random_program(rng, body_len=BODY_LEN):
    """Constrained-random program over the architectural subset.

    Pin behavior is in random_program_pins. This subset exercises every
    shift/FIFO/scratch/flow-control corner, the IRQ flags, WAIT on a
    flag, the computed jumps and the injected instruction of MOV EXEC.
    Backward jumps only on the self-bounding X--/Y-- conditions; a
    computed jump can go anywhere, and random_case drops a program that
    does not end. Ends with an X/Y dump epilogue and a park loop.
    """
    assert body_len == BODY_LEN
    body = []
    for i in range(body_len):
        kind = rng.random()
        if kind < 0.14:
            ins = SET(rng.choice([SDST_X, SDST_Y]), rng.randrange(32))
        elif kind < 0.34:
            ins = MOV(rng.choice([MDST_X, MDST_Y, MDST_ISR, MDST_OSR] * 3
                                 + [MDST_PC, MDST_EXEC]),
                      rng.choice([SRC_X, SRC_Y, SRC_NULL, SRC_STATUS,
                                  SRC_ISR, SRC_OSR]),
                      rng.choice([M_COPY, M_INV, M_REV]))
        elif kind < 0.48:
            ins = IN_(rng.choice([SRC_X, SRC_Y, SRC_NULL, SRC_ISR,
                                  SRC_OSR]), rng.randrange(1, 9))
        elif kind < 0.62:
            ins = OUT(rng.choice([ODST_X, ODST_Y, ODST_NULL, ODST_ISR] * 4
                                 + [ODST_PC]), rng.randrange(1, 9))
        elif kind < 0.72:
            ins = PUSH(iffull=rng.randrange(2), block=rng.randrange(2))
        elif kind < 0.82:
            ins = PULL(ifempty=rng.randrange(2), block=rng.randrange(2))
        elif kind < 0.91:
            cond = rng.choice([C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC,
                               C_XNEY, C_NOTOSRE])
            if cond in (C_XDEC, C_YDEC):
                target = rng.randrange(0, PARK + 1)
            else:
                target = rng.randrange(i + 1, PARK + 1)
            ins = JMP(target, cond)
        elif kind < 0.96:
            action = rng.random()
            ins = IRQ(rng.randrange(4), clr=int(0.45 <= action < 0.85),
                      wait=int(action >= 0.85))
        else:
            ins = WAIT(rng.randrange(2), W_IRQ, rng.randrange(4))
        body.append(_with_delay(rng, ins, 5))
    return _finish(body)


def _run(g, max_steps, max_ticks):
    """Run the model to its end. False when the program does not end, or
    needs more ticks than a simulation should spend on one program."""
    steps = 0
    while steps < max_steps and not g.done():
        g.step()
        steps += 1
    if g.done() and not g.parked:
        # The hardware does not stop at the park loop: it runs the loop's
        # JMP for ever, and that JMP has side-set bits like any other
        # instruction (all 0). Run it once, so that the model's pins end
        # where the hardware's do.
        g.step()
    return (g.done() and g.ticks <= max_ticks), steps


def random_case(seed, max_steps=2000, max_ticks=3000):
    """One terminating constrained-random case: (program, cfg, tx, golden).

    Reseeds until the golden model ends within max_steps, so the
    hardware comparison point is well defined.
    """
    while True:
        rng = random.Random(seed)
        prog = random_program(rng)
        cfg = dict(shiftctrl=rng.randrange(16), thresh=rng.randrange(256),
                   status_cfg=rng.randrange(32))
        tx = [rng.randrange(256) for _ in range(rng.randrange(7))]
        g = GoldenSM(prog, cfg['shiftctrl'], cfg['thresh'],
                     cfg['status_cfg'], tx)
        ok, steps = _run(g, max_steps, max_ticks)
        if ok:
            return prog, cfg, tx, g, steps, seed
        seed += 7919  # does not end (nested loops, a jump back): resample


def random_program_pins(rng, cfg, body_len=BODY_LEN):
    """Random program over the pin-extended subset: SET/OUT/MOV to pins
    and pindirs, IN/MOV from pins, JMP PIN, WAIT on a pin, the computed
    jumps, MOV EXEC, and random side-set bits (decoded by the model from
    the instruction word, exactly like RTL)."""
    assert body_len == BODY_LEN
    cnt, opt = cfg["side_count"], cfg["side_opt"]
    delay_bits = 5 - cnt - (1 if (cnt and opt) else 0)

    def sbits():
        if cnt == 0:
            return 0
        if opt and rng.random() < 0.5:
            return 0  # optional side-set left off: pins untouched
        return side(rng.randrange(1 << cnt), cnt, bool(opt))

    body = []
    for i in range(body_len):
        kind = rng.random()
        if kind < 0.19:
            ins = SET(rng.choice([SDST_PINS, SDST_PINDIRS, SDST_X,
                                  SDST_Y]), rng.randrange(32),
                      side=sbits())
        elif kind < 0.38:
            ins = MOV(rng.choice([MDST_PINS, MDST_X, MDST_Y, MDST_ISR,
                                  MDST_OSR] * 3 + [MDST_PC, MDST_EXEC]),
                      rng.choice([SRC_PINS, SRC_X, SRC_Y, SRC_NULL,
                                  SRC_STATUS, SRC_ISR, SRC_OSR]),
                      rng.choice([M_COPY, M_INV, M_REV]),
                      side=sbits())
        elif kind < 0.52:
            ins = IN_(rng.choice([SRC_PINS, SRC_X, SRC_Y, SRC_NULL,
                                  SRC_ISR, SRC_OSR]),
                      rng.randrange(1, 9), side=sbits())
        elif kind < 0.66:
            ins = OUT(rng.choice([ODST_PINS, ODST_PINDIRS, ODST_X,
                                  ODST_Y, ODST_NULL, ODST_ISR] * 4
                                 + [ODST_PC]),
                      rng.randrange(1, 9), side=sbits())
        elif kind < 0.74:
            ins = PUSH(iffull=rng.randrange(2), block=rng.randrange(2),
                       side=sbits())
        elif kind < 0.82:
            ins = PULL(ifempty=rng.randrange(2), block=rng.randrange(2),
                       side=sbits())
        elif kind < 0.94:
            cond = rng.choice([C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC,
                               C_XNEY, C_PIN, C_NOTOSRE])
            if cond in (C_XDEC, C_YDEC):
                target = rng.randrange(0, PARK + 1)
            else:
                target = rng.randrange(i + 1, PARK + 1)
            ins = JMP(target, cond, side=sbits())
        else:
            ins = WAIT(rng.randrange(2), rng.choice([W_GPIO, W_PIN]),
                       rng.randrange(16), side=sbits())
        body.append(_with_delay(rng, ins, delay_bits))
    return _finish(body)


def random_case_pins(seed, pins, dirs, max_steps=2000, max_ticks=3000):
    """One terminating pin-extended case. pins/dirs carry the pad state
    left by the previous seed (project-level registers survive restarts)."""
    while True:
        rng = random.Random(seed)
        cfg = dict(
            shiftctrl=rng.randrange(16),
            thresh=rng.randrange(256),
            status_cfg=rng.randrange(32),
            out_base=rng.randrange(16), out_count=rng.randrange(9),
            set_base=rng.randrange(16), set_count=rng.randrange(8),
            in_base=rng.randrange(16),
            side_base=rng.randrange(16), side_count=rng.randrange(3),
            side_opt=rng.randrange(2),
            jmp_pin=rng.randrange(16),
        )
        prog = random_program_pins(rng, cfg)
        tx = [rng.randrange(256) for _ in range(rng.randrange(7))]
        g = GoldenSM(prog, cfg["shiftctrl"], cfg["thresh"],
                     cfg["status_cfg"], tx, pin_cfg=cfg,
                     pins=pins, dirs=dirs)
        ok, steps = _run(g, max_steps, max_ticks)
        if ok:
            return prog, cfg, tx, g, steps, seed
        seed += 7919
