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
cycle-exact. Each SM has a fractional (16.8) clock divider, 8-bit OSR/ISR
shift registers with configurable direction plus autopull/autopush, X/Y
scratch registers, and 4-deep TX/RX FIFOs.

The host talks to the chip over a mode-0 SPI slave: one command byte
`{RW, ADDR[6:0]}` followed by data bytes with address auto-increment
(held for FIFO registers). The register map covers instruction memory,
per-SM configuration (clock divider, wrap region, shift control, pin
mapping), FIFO access, GPIO readback, and IRQ flags.

The 16-entry GPIO space maps to: 8 bidirectional pins (`uio`, direction
controlled at runtime via `SET/OUT PINDIRS`), 5 input-only pins
(`ui[7:3]`), and 3 output-only pins (`uo[4:2]`). A per-pin priority
router lets both SMs drive disjoint (or shared) pin sets; pins hold
their last driven value.

On restart a state machine begins executing at its `WRAP_BOTTOM`, so the
two SMs can run independent programs in the shared memory (e.g. UART TX
and UART RX simultaneously, full duplex).

## How to test

`test/metastable.py` contains a Python ISA assembler and SPI host driver;
`test/test.py` shows the full flow. The included UART loopback test:
SM0 runs an 8N1 UART transmitter on GPIO0, SM1 runs a receiver on the
same pin, bytes pushed into SM0's TX FIFO over SPI come back out of
SM1's RX FIFO.

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
