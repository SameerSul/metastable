#!/usr/bin/env bash
# Formal verification (yosys-smtbmc + an SMT solver).
#   pio_fifo:   structural invariants + in-order data integrity
#   pio_clkdiv: exact tick spacing (div_int + accumulator carry), so 256
#               ticks span exactly 256*div_int + div_frac cycles
# BMC catches any reachable violation up to 24 cycles; k-induction then
# proves the properties for unbounded time.
set -euo pipefail
cd "$(dirname "$0")/.."

SOLVER="${SOLVER:-z3}"
mkdir -p formal/out

for top in pio_fifo pio_clkdiv; do
  echo "==== $top ===="
  yosys -q -p "
    read_verilog -formal -DFORMAL src/$top.v
    prep -top $top -nordff
    flatten
    memory -nomap -nordff
    opt -fast
    async2sync
    dffunmap
    write_smt2 -wires formal/out/$top.smt2
  "
  echo "-- BMC (24 cycles)"
  yosys-smtbmc -s "$SOLVER" -t 24 --dump-vcd "formal/out/${top}_cex.vcd" "formal/out/$top.smt2"
  echo "-- k-induction"
  yosys-smtbmc -s "$SOLVER" -i -t 24 --dump-vcd "formal/out/${top}_cex.vcd" "formal/out/$top.smt2"
  echo "$top: PROVED"
done

echo "all formal properties PROVED"
