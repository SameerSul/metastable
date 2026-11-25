#!/usr/bin/env bash
# Formal verification (yosys-smtbmc + an SMT solver).
#   pio_fifo:   structural invariants + in-order data integrity
#   pio_clkdiv: exact tick spacing (div_int + accumulator carry), so 256
#               ticks span exactly 256*div_int + div_frac cycles
#   spi_slave:  register-bus strobe discipline under arbitrary pad
#               waveforms (single-cycle, exclusive, exactly-once)
#   pio_sm:     the engine never pops an empty TX FIFO or pushes into a
#               full RX FIFO, and its shift counters stay in 0..8, for
#               every instruction word, configuration and input
#   top level:  the pin router, with the whole design around it: a pin
#               takes the value of the engine that writes it, the second
#               engine wins when both write, and a pin that nobody writes
#               holds; the same for the pin directions
# BMC catches any reachable violation up to 24 cycles; k-induction then
# proves the properties for unbounded time.
#
# At the top level the two parts do different work. An engine is off
# until the host enables it, and the host cannot write a register in the
# first 35 clock cycles after reset: a bounded check on the SPI block
# shows that. So no engine runs within the 24 cycles of the bounded
# check. There it covers the reset state, and the induction step covers
# every other state, reachable or not.
set -euo pipefail
cd "$(dirname "$0")/.."

SOLVER="${SOLVER:-z3}"
mkdir -p formal/out

prove() { # name, smt2 file
  echo "-- BMC (24 cycles)"
  yosys-smtbmc -s "$SOLVER" -t 24 --dump-vcd "formal/out/${1}_cex.vcd" "$2"
  echo "-- k-induction"
  yosys-smtbmc -s "$SOLVER" -i -t 24 --dump-vcd "formal/out/${1}_cex.vcd" "$2"
  echo "$1: PROVED"
}

for top in pio_fifo pio_clkdiv spi_slave pio_sm; do
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
  prove "$top" "formal/out/$top.smt2"
done

# The pin router is part of the top module. FORMAL_TOP turns on its
# properties alone: the blocks' own properties (FORMAL) are proved above.
echo "==== pin router (tt_um_sulems6_metastable) ===="
yosys -q -p "
  read_verilog -formal -DFORMAL_TOP src/project.v src/spi_slave.v src/pio_sm.v src/pio_fifo.v src/pio_clkdiv.v
  prep -top tt_um_sulems6_metastable -nordff
  flatten
  memory -nomap -nordff
  opt -fast
  async2sync
  dffunmap
  write_smt2 -wires formal/out/pin_router.smt2
"
prove pin_router formal/out/pin_router.smt2

echo "all formal properties PROVED"
