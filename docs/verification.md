# How Metastable is verified

Metastable is checked in five ways, and each one shows a different thing. This document states what each check covers, what it leaves out, and how to run it.

| Check | What it shows | Size | Result |
| --- | --- | --- | --- |
| [Directed tests](#directed-tests) | Each protocol program produces a correct waveform | 26 tests | Pass on the RTL and on the netlist |
| [Example tests](#example-tests) | The software that ships works, driver and example together | 11 tests | Pass on the RTL and on the netlist |
| [Random co-simulation](#random-co-simulation) | The engine executes every instruction form as the reference model does | 2 tests: 20 seeds on a push, 200 in the long run | All seeds agree |
| [Formal proofs](#formal-proofs) | Five blocks keep their invariants for every input | 26 assertions on 5 targets | Proved |
| [Netlist equivalence](#netlist-equivalence) | The layout’s netlist computes what the RTL computes | 1,300 signal pairs | Proved for both 8x4 netlists |

The netlist results belong to the layouts of the 8x4 port: workflow run 44693403556 at placement density 75, and run 44693403526 at density 60. They have to run again on the netlist that is submitted.

## What the terms mean

Five terms recur in this document:

- **RTL**: the Verilog source in `src/`
- **Netlist**: the gates and flip-flops that the layout flow wrote after place and route, `runs/wokwi/final/nl/tt_um_sulems6_metastable.nl.v`
- **Engine**: one of the two programmable state machines, `pio_sm`
- **Host**: whatever drives the SPI port. On the demo board it is the RP2040.
- **Reference model**: `test/golden.py`, a Python model of one engine

## Directed tests

`test/test.py` holds 28 tests that run in Icarus Verilog under cocotb, at a 50 MHz clock. Two of them are the random co-simulation, which has a section of its own. The other 26 are directed.

Seventeen directed tests run protocol programs:

| Protocol | How the test checks it |
| --- | --- |
| UART | Loopback: one engine sends frames, and the other receives them on the same pin |
| SPI, mode 0 | Loopback: a master on one engine and a slave on the other exchange bytes in both directions |
| SPI, modes 0 to 3, both roles | A monitor that knows only the mode checks the idle level of SCK, the sampling edge, and that both data lines hold still around it |
| I2C write | A bus model with a target that acknowledges each byte |
| I2C read, 100 kHz and 400 kHz | A bus model checks every byte and acknowledge, a clock stretch by the target and a missing device. It measures the bus times against the minimums that the test file lists. |
| Manchester | Loopback: the receiving engine locks to the preamble and decodes the frame |
| WS2812 | A decoder checks every pulse width against the datasheet windows |
| USB low-speed transmit | A decoder checks sync, NRZI, bit stuffing, CRC16, end of packet and the measured bit rate |
| PS/2 device | A host model checks framing, parity and the clock band |
| NEC infrared receive | A transmitter model sends a frame, and the engine decodes address and command from the gap lengths |
| Capture and replay | Loopback: one engine replays a waveform, and the other samples it into a snapshot |
| Two protocols at once | The USB decoder and the WS2812 decoder, on the two engines in the same run |

A decoder or a bus model shares no code with the program that it checks. A loopback checks two programs against each other, so it can miss a fault that both share.

The other nine directed tests cover the register map, the instruction set’s corners, the SPI port and reset:

| Test | What it covers |
| --- | --- |
| `test_registers` | Reads and writes of the registers and of the instruction memory over SPI |
| `test_mov_ops` | MOV with invert, reverse, STATUS and EXEC |
| `test_irq_handshake` | The IRQ handshake between the two engines, and the host interrupt pin |
| `test_jmp_variants` | WAIT on a pin, three JMP conditions and three OUT destinations |
| `test_fifo_semantics` | A non-blocking PULL, the PUSH threshold and backpressure on the RX FIFO |
| `test_clkdiv_restart` | The fractional clock divider, and a restart in the middle of a run |
| `test_spi_fuzz` | Random read and write bursts on the instruction memory, with transfers that CS_N cuts at a random bit. A reference model predicts the memory: only a byte that is clocked in full takes effect. |
| `test_out_only_pads` | An engine drives the three pins that have no input path. The test checks the pads themselves, and that they hold. |
| `test_reload_from_unknown` | Reset and reload from an unknown state: see [Gate-level simulation](#gate-level-simulation) |

These tests do not cover the pad cells, and they do not cover timing. The I2C minimums are the values in `test/metastable.py`, which names the I2C-bus specification as their source.

## Example tests

`test/test_examples.py` runs each example of `sw/examples` the way a user runs it on the demo board. The example’s own host function calls the driver in `sw/metastable_host.py`, which bit-bangs SPI through a stand-in for the board’s library. The stand-in puts the simulated chip behind the board’s pins.

The stand-in’s host is slow on purpose: 30 clock cycles for each pin write. A real host is far slower than any protocol here. An example that works only when the host keeps up with the line fails in this file. Three examples failed that way when the tests were written, and they now load the FIFO before they start the engine.

Two examples have no test of their host side, because they have no host function: `usb_ls_tx.py` and `two_protocols.py` need a host that paces its writes. Their programs run in the directed tests.

The stand-in is a model of the demo board. No test here runs on a board.

## Random co-simulation

Two tests draw random programs and compare the engine with the reference model. The model has no clock: it executes one instruction at a time and predicts the end state. The two tests differ in what a seed draws and in what they compare:

- **Without pins**: a seed draws a 16-instruction program, the shift, threshold and status configuration, and up to 6 bytes for the TX FIFO. The test compares the program counter, the four IRQ flags and the exact contents of the RX FIFO. A program that runs to its end pushes X and Y there.
- **With pins**: a seed also draws the pin configuration: the pins of OUT, SET, IN and side-set, and the pin of JMP. The test compares the program counter, the RX FIFO and the readback of all 16 pins.

A push runs 12 seeds without pins and 8 with pins. The long run uses 120 and 80: `.github/workflows/cosim.yaml` runs it once a week and on request.

`test/cosim_coverage.py` prints which instruction forms the runs executed. The table has 105 cells: one for each operand value of the instruction set, and one for each outcome of an instruction that can stall. A cell counts only runs in which the hardware matched the model. The long run has to fill every cell, and it does:

| Run | Seeds | Instructions executed | Cells filled |
| --- | --- | --- | --- |
| On a push | 12 + 8 | 195 | 94 of 105 |
| Long | 120 + 80 | 3,476 | 105 of 105 |

Three cells of the long run are thin. A PUSH that drops its byte on a full FIFO ran once. An injected instruction that does not jump ran 4 times, and an autopush that parks on a full FIFO ran 5 times.

The comparison is of end states, so it does not check timing: the directed tests do that. It runs one engine at a time. The first long run failed on one seed, and the fault was in the model: the model stopped before the last instruction that the hardware runs.

## Formal proofs

`formal/run.sh` proves 26 assertions on five targets with yosys-smtbmc and Z3. Each target gets a bounded check from reset to depth 24, and then k-induction, which extends the result to every later cycle. The whole script takes under a minute.

| Target | What is proved | What is free in the proof |
| --- | --- | --- |
| `pio_fifo` | The count stays in 0 to 8. The full and empty flags and the two pointers agree with it. A byte that goes into a slot is the byte that comes out. | Every input |
| `pio_clkdiv` | Each gap between two ticks is the integer divider plus the carry of an 8-bit accumulator. It follows that 256 ticks take exactly 256 times the integer part plus the fraction. | The divider value, held stable, with the divider enabled and not restarted |
| `spi_slave` | A write strobe and a read strobe never come together, and each lasts one cycle. A request or a strobe exists only right after a byte ends, which rules out a second strobe for the same byte. | The three SPI pads, in any waveform, glitches and cut transfers included |
| `pio_sm` | The engine never pops an empty TX FIFO and never pushes into a full RX FIFO. Its two shift counters stay in 0 to 8. | The instruction word, the configuration, the tick, the pins and the FIFO flags |
| Pin router | A pin takes the value of the engine that writes it. The second engine wins when both write. A pin that no engine writes holds. The same for the pin directions. | The pads. The proof runs on the whole design, with both engines. |

Each property of the last two targets was also checked the other way: nine faults put in on purpose make the proofs fail.

At the top level the bounded check and the induction do different work. The host cannot write a register in the first 35 clock cycles after reset, so no engine runs within the depth of the bounded check. The check covers the reset state there, and the induction step covers every other state.

The proofs are about invariants. No proof says what an instruction computes: the random co-simulation covers that. The clock divider’s proof assumes a stable configuration. It is about a divider that runs freely, and a host changes the divider while the engine is off.

## Gate-level simulation

The whole suite, 39 tests, passes on the netlist of run 44693403556. The simulation uses the kit’s Verilog models of the cells. It is functional: it has no cell delays and no wire delays, so it says nothing about timing.

One test is written for the netlist. `test_reload_from_unknown` first makes all 1,294 flip-flops unknown. It toggles the notifier of each flip-flop cell, which is how the kit’s models answer a timing violation. That stands for a design right after power-up.

Then the test holds reset for 10 clock cycles, loads a program over SPI, runs it and reads its two result bytes. It does that three times, with a different engine and pin pair each time. It samples the 24 output pins in every clock cycle after reset. All three rounds pass, and the test fails when the reset is left out.

One output is unknown in one case. In the read that empties an RX FIFO, the SPI port fetches the next byte ahead of the host. After the last byte, that is the next word of the FIFO storage, which nothing has written since reset. Its top bit shows on MISO until CS_N rises: 10 clock cycles at the test’s SPI speed.

The test checks that every such sample comes after the host’s last sampling edge. On silicon the pin shows one bit of whatever the storage holds. A read of an empty RX FIFO returns such a word.

## Netlist equivalence

`formal/netlist_equiv.py` proves that the netlist computes what the RTL computes. The gate-level simulation shows that for the tests that exist. The proof covers every state and every input.

The script gives each flip-flop of the netlist a partner in the RTL. Then it proves one statement. Suppose that every flip-flop holds the value of its partner, and that both designs get the same inputs. Then every output pin has the same value in both, and after the next clock edge every flip-flop again holds the value of its partner.

A netlist has 1,294 flip-flops, and each gets its partner in one of three ways:

| Flip-flops | How the script finds the partner |
| --- | --- |
| 1,253 | The net at the output has the name of a wire in the RTL |
| 19 | The flip-flop drives an output pin |
| 22 | The script proves, in the netlist alone, that the flip-flop’s data input equals that of a named flip-flop. These are the copies that synthesis makes of memory read addresses: the read pointers of the four FIFOs and the two program counters. |

All 1,300 pairs are proven for both 8x4 netlists, in under 7 minutes each: 24 output bits, 1,253 named flip-flops, the 22 copies and one constant. The constant is bit 0 of the SPI output shift register. Every assignment gives it 0, and the netlist has no flip-flop for it. With `--control` the script changes one gate of the netlist, and the proof of that output fails.

The proof has four limits:

- **The start**: the pairs have to agree once. Two clock edges of reset do that for every register with a reset value. The instruction memory and the FIFO storage have none, and the proof takes them to start equal.
- **The cell functions**: the proof reads the function of each cell from the kit’s Liberty file. It does not compare those functions with the cell layouts or with the kit’s Verilog models. The gate-level simulation uses the Verilog models.
- **Time**: the proof says nothing about setup, hold or clock skew. The timing reports of the layout flow cover those.
- **Open values**: where the tool leaves a value of the RTL open, the proof reads it as 0

## What no check covers

Five things are outside every check in this document:

- **Silicon**: no chip exists yet. Every result here is from simulation or from a proof.
- **Timing in simulation**: all simulations are functional. Timing closure rests on the static timing analysis of the layout flow.
- **The pads**: the tests drive and read the digital pins of the design. The pad cells and their electrical behavior are outside them.
- **The inputs’ synchronizers**: the simulations have no metastability. A flip-flop that samples a changing input takes the old value or the new one.
- **The demo board**: the examples run against a stand-in for the board’s library

## How to run the checks

The directed tests, the example tests and the push-sized random run, on the RTL:

```sh
(cd test && make -B)
```

The same on a netlist, with `PDK_ROOT` set to the kit and the netlist copied to `test/gate_level_netlist.v`:

```sh
(cd test && make -B GATES=yes)
```

The long random run and its coverage table:

```sh
(cd test && COSIM_ARCH_SEEDS=120 COSIM_PIN_SEEDS=80 \
    COCOTB_TEST_FILTER=test_random_cosim make -B)
python3 test/cosim_coverage.py --check
```

The formal proofs:

```sh
./formal/run.sh
```

The netlist equivalence. Add `--control` for the run that has to fail:

```sh
NETLIST=runs/wokwi/final/nl/tt_um_sulems6_metastable.nl.v
LIB=$PDK_ROOT/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib
formal/netlist_equiv.py --netlist $NETLIST \
    --liberty $LIB/sg13cmos5l_stdcell_typ_1p20V_25C.lib
```

The tools are Icarus Verilog, cocotb 2.0, Yosys with yosys-smtbmc, and Z3.
