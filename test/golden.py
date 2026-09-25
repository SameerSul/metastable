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
    JMP, IN_, OUT, PUSH, PULL, MOV, IRQ, SET, side,
    SRC_PINS, SRC_X, SRC_Y, SRC_NULL, SRC_STATUS, SRC_ISR, SRC_OSR,
    ODST_PINS, ODST_X, ODST_Y, ODST_NULL, ODST_PINDIRS, ODST_ISR,
    MDST_PINS, MDST_X, MDST_Y, MDST_ISR, MDST_OSR, M_COPY, M_INV, M_REV,
    SDST_PINS, SDST_X, SDST_Y, SDST_PINDIRS,
    C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC, C_XNEY, C_PIN, C_NOTOSRE,
)


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

    def _apply_side(self, ci):
        en, val = self._side(ci)
        if en:
            self._write_pins(self.pc_cfg.get("side_base", 0),
                             self.pc_cfg.get("side_count", 0), val)

    def _src(self, code):
        return {SRC_PINS: self._pins_in8(), SRC_X: self.x, SRC_Y: self.y,
                SRC_NULL: 0, SRC_STATUS: self._status(), SRC_ISR: self.isr,
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
                    C_PIN: bool((self._gpio_in()
                                 >> self.pc_cfg.get("jmp_pin", 0)) & 1),
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
                    self._apply_side(ci)
                    self.parked = True  # stall on full RX
                    return False
                self.rx.append(shifted)
                self.isr, self.isr_cnt = 0, 0
            else:
                self.isr, self.isr_cnt = shifted, cnt

        elif opc == 3:  # OUT
            refill = self.autopull and self._osr_empty()
            if refill and not self.tx:
                self._apply_side(ci)
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
            elif dst == ODST_ISR:
                self.isr, self.isr_cnt = bits, n

        elif opc == 4:  # PUSH / PULL
            block = (ci >> 5) & 1
            iffull = (ci >> 6) & 1
            if (ci >> 7) & 1:  # PULL
                if not iffull or self._osr_empty():
                    if not self.tx:
                        if block:
                            self._apply_side(ci)
                            self.parked = True
                            return False
                        self.osr, self.osr_cnt = self.x, 0
                    else:
                        self.osr, self.osr_cnt = self.tx.pop(0), 0
            else:  # PUSH
                if not iffull or self.isr_cnt >= self.push_eff:
                    if len(self.rx) == 8:
                        if block:
                            self._apply_side(ci)
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
            if dst == MDST_PINS:
                self._write_pins(self.pc_cfg.get("out_base", 0),
                                 self.pc_cfg.get("out_count", 0), val)
            elif dst == MDST_X:
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


def random_program_pins(rng, cfg, body_len=16):
    """Random program over the pin-extended subset: SET/OUT/MOV to pins
    and pindirs, IN/MOV from pins, JMP PIN, and random side-set bits
    (decoded by the model from the instruction word, exactly like RTL)."""
    def sbits():
        cnt, opt = cfg["side_count"], cfg["side_opt"]
        if cnt == 0:
            return 0
        if opt and rng.random() < 0.5:
            return 0  # optional side-set left off: pins untouched
        return side(rng.randrange(1 << cnt), cnt, bool(opt))

    body = []
    park = body_len + 4
    for i in range(body_len):
        kind = rng.random()
        if kind < 0.20:
            body.append(SET(rng.choice([SDST_PINS, SDST_PINDIRS, SDST_X,
                                        SDST_Y]), rng.randrange(32),
                            side=sbits()))
        elif kind < 0.40:
            body.append(MOV(rng.choice([MDST_PINS, MDST_X, MDST_Y, MDST_ISR,
                                        MDST_OSR]),
                            rng.choice([SRC_PINS, SRC_X, SRC_Y, SRC_NULL,
                                        SRC_STATUS, SRC_ISR, SRC_OSR]),
                            rng.choice([M_COPY, M_INV, M_REV]),
                            side=sbits()))
        elif kind < 0.55:
            body.append(IN_(rng.choice([SRC_PINS, SRC_X, SRC_Y, SRC_NULL,
                                        SRC_ISR, SRC_OSR]),
                            rng.randrange(1, 9), side=sbits()))
        elif kind < 0.70:
            body.append(OUT(rng.choice([ODST_PINS, ODST_PINDIRS, ODST_X,
                                        ODST_Y, ODST_NULL, ODST_ISR]),
                            rng.randrange(1, 9), side=sbits()))
        elif kind < 0.78:
            body.append(PUSH(iffull=rng.randrange(2), block=rng.randrange(2),
                             side=sbits()))
        elif kind < 0.86:
            body.append(PULL(ifempty=rng.randrange(2), block=rng.randrange(2),
                             side=sbits()))
        else:
            cond = rng.choice([C_ALWAYS, C_NOTX, C_XDEC, C_NOTY, C_YDEC,
                               C_XNEY, C_PIN, C_NOTOSRE])
            if cond in (C_XDEC, C_YDEC):
                target = rng.randrange(0, park + 1)
            else:
                target = rng.randrange(i + 1, park + 1)
            body.append(JMP(target, cond, side=sbits()))
    dump = [MOV(MDST_ISR, SRC_X), PUSH(block=0),
            MOV(MDST_ISR, SRC_Y), PUSH(block=0)]
    return body + dump + [JMP(park)]


def random_case_pins(seed, pins, dirs, max_steps=2000):
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
        steps = 0
        park = len(prog) - 1
        while steps < max_steps and not g.parked and g.pc != park:
            g.step()
            steps += 1
        if g.parked or g.pc == park:
            return prog, cfg, tx, g, steps, seed
        seed += 7919
