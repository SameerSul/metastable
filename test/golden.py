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

import random

from metastable_asm import (
    JMP, IN_, OUT, PUSH, PULL, MOV, IRQ, SET,
    SRC_X, SRC_Y, SRC_NULL, SRC_STATUS, SRC_ISR, SRC_OSR,
    ODST_X, ODST_Y, ODST_NULL, ODST_ISR,
    MDST_X, MDST_Y, MDST_ISR, MDST_OSR, M_COPY, M_INV, M_REV,
    SDST_X, SDST_Y,
    C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC, C_XNEY, C_NOTOSRE,
)


def _rev8(v):
    return int(f"{v & 0xFF:08b}"[::-1], 2)


class GoldenSM:
    def __init__(self, program, shiftctrl, thresh, status_cfg, tx_fifo):
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

    # -- helpers matching the RTL scratch logic --------------------------
    def _osr_empty(self):
        return self.osr_cnt >= self.pull_eff

    def _status(self):
        level = len(self.rx) if self.status_cfg & 0x10 else len(self.tx)
        return 0xFF if level < (self.status_cfg & 0xF) else 0x00

    def _src(self, code):
        return {SRC_X: self.x, SRC_Y: self.y, SRC_NULL: 0,
                SRC_STATUS: self._status(), SRC_ISR: self.isr,
                SRC_OSR: self.osr}[code]

    def step(self):
        """Execute one instruction; returns False once parked."""
        if self.parked:
            return False
        ci = self.prog[self.pc]
        opc = ci >> 13
        n = 8 if (ci & 0x1F) == 0 or (ci & 0x1F) > 8 else ci & 0xF
        mask = 0xFF >> (8 - n)
        jump = None

        if opc == 0:  # JMP
            cond = (ci >> 5) & 7
            take = {C_ALWAYS: True, C_NOTX: self.x == 0,
                    C_XDEC: self.x != 0, C_NOTY: self.y == 0,
                    C_YDEC: self.y != 0, C_XNEY: self.x != self.y,
                    C_NOTOSRE: not self._osr_empty()}[cond]
            if cond == C_XDEC:
                self.x = (self.x - 1) & 0xFF
            if cond == C_YDEC:
                self.y = (self.y - 1) & 0xFF
            if take:
                jump = ci & 0x1F

        elif opc == 2:  # IN
            src = (ci >> 5) & 7
            val = self._src(src) & mask
            if self.in_right:
                shifted = ((self.isr >> n) | (val << (8 - n))) & 0xFF
            else:
                shifted = ((self.isr << n) | val) & 0xFF
            cnt = min(self.isr_cnt + n, 8)
            if self.autopush and cnt >= self.push_eff:
                if len(self.rx) == 8:
                    self.parked = True  # stall on full RX
                    return False
                self.rx.append(shifted)
                self.isr, self.isr_cnt = 0, 0
            else:
                self.isr, self.isr_cnt = shifted, cnt

        elif opc == 3:  # OUT
            refill = self.autopull and self._osr_empty()
            if refill and not self.tx:
                self.parked = True
                return False
            osr_eff = self.tx.pop(0) if refill else self.osr
            cnt_eff = 0 if refill else self.osr_cnt
            if self.out_right:
                bits = osr_eff & mask
                self.osr = osr_eff >> n
            else:
                bits = (osr_eff >> (8 - n)) & 0xFF
                self.osr = (osr_eff << n) & 0xFF
            self.osr_cnt = min(cnt_eff + n, 8)
            dst = (ci >> 5) & 7
            if dst == ODST_X:
                self.x = bits
            elif dst == ODST_Y:
                self.y = bits
            elif dst == ODST_ISR:
                self.isr, self.isr_cnt = bits, n

        elif opc == 4:  # PUSH / PULL
            block = (ci >> 5) & 1
            iffull = (ci >> 6) & 1
            if (ci >> 7) & 1:  # PULL
                if not iffull or self._osr_empty():
                    if not self.tx:
                        if block:
                            self.parked = True
                            return False
                        self.osr, self.osr_cnt = self.x, 0
                    else:
                        self.osr, self.osr_cnt = self.tx.pop(0), 0
            else:  # PUSH
                if not iffull or self.isr_cnt >= self.push_eff:
                    if len(self.rx) == 8:
                        if block:
                            self.parked = True
                            return False
                        self.isr, self.isr_cnt = 0, 0
                    else:
                        self.rx.append(self.isr)
                        self.isr, self.isr_cnt = 0, 0

        elif opc == 5:  # MOV
            val = self._src(ci & 7)
            op = (ci >> 3) & 3
            if op == M_INV:
                val = (~val) & 0xFF
            elif op == M_REV:
                val = _rev8(val)
            dst = (ci >> 5) & 7
            if dst == MDST_X:
                self.x = val
            elif dst == MDST_Y:
                self.y = val
            elif dst == MDST_ISR:
                self.isr, self.isr_cnt = val, 0
            elif dst == MDST_OSR:
                self.osr, self.osr_cnt = val, 0

        elif opc == 6:  # IRQ (set/clear only in the generated subset)
            if (ci >> 6) & 1:
                self.irq[ci & 3] = 0
            else:
                self.irq[ci & 3] = 1

        elif opc == 7:  # SET
            dst = (ci >> 5) & 7
            if dst == SDST_X:
                self.x = ci & 0x1F
            elif dst == SDST_Y:
                self.y = ci & 0x1F

        self.pc = jump if jump is not None else (self.pc + 1) & 0x1F
        return True


def random_program(rng, body_len=16):
    """Constrained-random program over the architectural subset.

    Pin, WAIT, EXEC and delay behavior is covered by the directed tests;
    this subset exercises every shift/FIFO/scratch/flow-control corner.
    Backward jumps only on the self-bounding X--/Y-- conditions, so
    programs terminate. Ends with an X/Y dump epilogue and a park loop.
    """
    body = []
    park = body_len + 4
    for i in range(body_len):
        kind = rng.random()
        if kind < 0.15:
            body.append(SET(rng.choice([SDST_X, SDST_Y]), rng.randrange(32)))
        elif kind < 0.35:
            body.append(MOV(rng.choice([MDST_X, MDST_Y, MDST_ISR, MDST_OSR]),
                            rng.choice([SRC_X, SRC_Y, SRC_NULL, SRC_STATUS,
                                        SRC_ISR, SRC_OSR]),
                            rng.choice([M_COPY, M_INV, M_REV])))
        elif kind < 0.50:
            body.append(IN_(rng.choice([SRC_X, SRC_Y, SRC_NULL, SRC_ISR,
                                        SRC_OSR]), rng.randrange(1, 9)))
        elif kind < 0.65:
            body.append(OUT(rng.choice([ODST_X, ODST_Y, ODST_NULL,
                                        ODST_ISR]), rng.randrange(1, 9)))
        elif kind < 0.75:
            body.append(PUSH(iffull=rng.randrange(2), block=rng.randrange(2)))
        elif kind < 0.85:
            body.append(PULL(ifempty=rng.randrange(2), block=rng.randrange(2)))
        elif kind < 0.95:
            cond = rng.choice([C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC,
                               C_XNEY, C_NOTOSRE])
            if cond in (C_XDEC, C_YDEC):
                target = rng.randrange(0, park + 1)
            else:
                target = rng.randrange(i + 1, park + 1)
            body.append(JMP(target, cond))
        else:
            body.append(IRQ(rng.randrange(4), clr=rng.randrange(2)))
    dump = [MOV(MDST_ISR, SRC_X), PUSH(block=0),
            MOV(MDST_ISR, SRC_Y), PUSH(block=0)]
    return body + dump + [JMP(park)]


def random_case(seed, max_steps=2000):
    """One terminating constrained-random case: (program, cfg, tx, golden).

    Reseeds until the golden model parks within max_steps, so the
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
        steps = 0
        park = len(prog) - 1
        while steps < max_steps and not g.parked and g.pc != park:
            g.step()
            steps += 1
        if g.parked or g.pc == park:
            return prog, cfg, tx, g, steps, seed
        seed += 7919  # non-terminating (nested bounded loops): resample
