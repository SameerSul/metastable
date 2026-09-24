#!/usr/bin/env bash
# Formal verification of pio_fifo (yosys-smtbmc + an SMT solver).
# BMC catches any reachable violation up to 24 cycles; k-induction then
# proves the properties for unbounded time.
set -euo pipefail
cd "$(dirname "$0")/.."

SOLVER="${SOLVER:-z3}"
mkdir -p formal/out

yosys -q -p "
  read_verilog -formal -DFORMAL src/pio_fifo.v
  prep -top pio_fifo -nordff
  flatten
  memory -nomap -nordff
  opt -fast
  async2sync
  dffunmap
  write_smt2 -wires formal/out/pio_fifo.smt2
"

echo "== BMC (24 cycles) =="
yosys-smtbmc -s "$SOLVER" -t 24 --dump-vcd formal/out/fifo_cex.vcd formal/out/pio_fifo.smt2

echo "== k-induction =="
yosys-smtbmc -s "$SOLVER" -i -t 24 --dump-vcd formal/out/fifo_cex.vcd formal/out/pio_fifo.smt2

echo "pio_fifo: all properties PROVED"
