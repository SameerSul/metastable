## How it works

Metastable is a general-purpose protocol emulator: two RP2040-PIO-inspired
programmable state machines that bit-bang protocols (UART, SPI, I2C, and
anything else that fits the timing) entirely in "firmware" loaded after
fabrication.

Each state machine executes one 16-bit instruction per divided-clock tick
from a shared 32-entry instruction memory:

| Opcode | Instruction | Function |
|--------|-------------|----------|
| 000 | `JMP cond, addr` | conditions: always, !X, X--, !Y, Y--, X!=Y, PIN, !OSRE |
| 001 | `WAIT pol, src, idx` | stall on GPIO / relative pin / IRQ flag |
| 010 | `IN src, n` | shift 1-8 bits into ISR (pins, X, Y, null, ISR, OSR) |
| 011 | `OUT dst, n` | shift 1-8 bits out of OSR (pins, X, Y, pindirs, PC, ISR) |
| 100 | `PUSH` / `PULL` | ISR -> RX FIFO / TX FIFO -> OSR, blocking or not |
| 101 | `MOV dst, op(src)` | copy/invert/bit-reverse; `MOV EXEC` injects a computed jump |
| 110 | `IRQ set/clr/wait idx` | 4 shared flags for inter-SM sync and host interrupts |
| 111 | `SET dst, imm` | drive pins/pindirs, load X/Y |

Every instruction carries a 5-bit delay/side-set field, so pin timing is
cycle-exact. Side-set can be made optional per instruction (an enable bit
replaces the top field bit) and can drive pin *directions* instead of
values, for open-drain protocols like I2C. Each SM has a fractional (16.8) clock divider, 8-bit OSR/ISR
shift registers with configurable direction plus autopull/autopush, X/Y
scratch registers, and 8-deep TX/RX FIFOs. `MOV STATUS` reads all-ones
while a configurable FIFO's fill level is below a per-SM threshold, for
flow-control in emulated protocols.

The host talks to the chip over a mode-0 SPI slave: one command byte
`{RW, ADDR[6:0]}` followed by data bytes with address auto-increment
(held for FIFO registers). The register map covers instruction memory,
per-SM configuration (clock divider, wrap region, shift control, pin
mapping), FIFO access and fill levels, GPIO readback, and IRQ flags.

The 16-entry GPIO space maps to: 8 bidirectional pins (`uio`, direction
controlled at runtime via `SET/OUT PINDIRS`), 5 input-only pins
(`ui[7:3]`), and 3 output-only pins (`uo[4:2]`). A per-pin priority
router lets both SMs drive disjoint (or shared) pin sets; pins hold
their last driven value.

On restart a state machine begins executing at its `WRAP_BOTTOM`, so the
two SMs can run independent programs in the shared memory (e.g. UART TX
and UART RX simultaneously, full duplex).

### Protocol reach

Beyond the UART/SPI/I2C baseline (all in the test suite), the test bench
demonstrates **Manchester coding** (IEEE 802.3 polarity): 8 ticks per
bit with a guaranteed mid-bit transition, receiver locks onto the
preamble's first mid-bit edge. The TX loop needs 4 ticks per half-bit,
so at 50 MHz / div 1 the ceiling is ~6 Mb/s Manchester — 10BASE-T's
20 Mbaud is out of reach, but 10BASE-T link pulses (100 us spacing) and
arbitrary sub-6 Mb/s Manchester links are emulatable.

**USB low-speed (1.5 Mb/s)** analysis: the fractional divider hits the
bit clock within USB's tolerance (div 4+43/256 = 8 ticks/bit, +0.03%).
TX is feasible today: drive D+/D- as a 2-bit symbol stream (J, K, SE0)
via `OUT PINS, 2` — the host pre-computes sync, NRZI, bit stuffing, CRC
and EOP, packing 4 symbols per FIFO byte. Consumption is 2.67 us per
FIFO byte vs ~1.3 us per byte for burst SPI writes at SCK = clk/8, so
the 8-deep FIFO never underruns. RX is not practical beyond short
captures: NRZI decode + bit unstuffing exceeds the two-SM instruction
budget, and a raw 4x-oversampled dump (6 MS/s x 2 pins) exceeds the SPI
drain rate.

## How to test

`sw/metastable_asm.py` is a Python ISA assembler that runs on CPython and
MicroPython; `sw/metastable_host.py` is a register-level host driver for
the demo board's RP2040 (bit-banged mode-0 SPI over `ui[2:0]`/`uo[0]`),
and `sw/examples/uart_loopback.py` runs the UART demo on silicon. The
cocotb suite imports the same assembler; `test/test.py` shows the full
flow with three protocol demos:

- **UART loopback**: SM0 runs an 8N1 transmitter on GPIO0, SM1 a
  receiver on the same pin; bytes pushed into SM0's TX FIFO over SPI
  come back out of SM1's RX FIFO.
- **SPI master/slave**: SM0 is a mode-0 master clocking SCK via
  optional side-set, SM1 a full-duplex slave; both directions verified
  MSB first with autopull/autopush.
- **I2C master**: SM0 does an open-drain address + data write (side-set
  routed to pin directions, output latches held low), against a Python
  slave model that ACKs each byte.
- **WS2812 / NeoPixel**: SM0 drives 800 kHz pulse-width-coded LED data
  (8 MHz fractional tick, 10 ticks/bit); every pulse is checked against
  the WS2812B datasheet windows and the GRB frame reassembled.
  `sw/examples/ws2812.py` runs it on silicon.

On the dev board, wire the RP2040 (or any SPI master, clk >= 8x SCK) to
`SPI_SCK/CS_N/MOSI` on `ui[2:0]` and `SPI_MISO` on `uo[0]`, then:

1. Write a program into instruction memory (registers 0x00-0x3F).
2. Configure the SM: clock divider, wrap region, pin bases/counts (0x50+/0x60+).
3. Set CTRL (0x40) enable + restart bits.
4. Stream data through the TX/RX FIFO registers; poll FSTAT (0x41) or
   use the HOST_IRQ pin (`uo[1]`) with the IRQ mask.

`uo[7]` outputs a clock heartbeat (clk/2^24) for bring-up, and
`uo[6:5]` expose the SMs' stall state for debugging.

## External hardware

Any SPI master as host (the Tiny Tapeout demo board's RP2040 works).
Protocol-under-test hardware connects to the GPIO pins: logic analyzer,
UART device, SPI/I2C peripherals, etc. I2C requires external pull-ups on
the chosen GPIO pins (drive low / release-to-input open-drain style).
