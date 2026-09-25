![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/formal/badge.svg) ![](../../workflows/fpga/badge.svg)

# Metastable: programmable protocol emulator

A general-purpose protocol emulator ASIC for the Jane Street protocol emulator
competition, built on [Tiny Tapeout](https://tinytapeout.com) (IHP 130nm CMOS5L,
6x4 tiles). Two RP2040-PIO-inspired programmable state machines bit-bang
arbitrary serial protocols — UART, SPI, I2C, and anything else that fits the
timing — entirely in firmware loaded over SPI after fabrication.

Full datasheet: [docs/info.md](docs/info.md)

| | |
|---|---|
| Protocols demonstrated | UART, SPI, I2C, Manchester, WS2812, USB LS (1.5 MHz, +299 ppm), PS/2, NEC IR — plus capture/replay and two protocols concurrently |
| Verification | 19 cocotb tests (independent bus/decoder models), constrained-random cosim vs a golden model, SPI fuzz with mid-byte aborts, formal proofs on 3 blocks (BMC + k-induction) |
| Timing theorem | proved: 256 divider ticks span exactly `256*div_int + div_frac` cycles — zero cumulative drift |
| Implementation | 6x4 tiles, 22% util, +1.45 ns setup @ slow corner after a placement audit recovered 4.3x margin, ~4.7 mW |
| Host software | MicroPython assembler + driver + examples for the TT demo board |

![Architecture](docs/architecture.png)

## Architecture

- 2 state machines, one 16-bit instruction per divided-clock tick, shared
  32-entry instruction memory
- Per SM: fractional 16.8 clock divider, 8-bit OSR/ISR with configurable shift
  direction + autopull/autopush, X/Y scratch registers, 8-deep TX/RX FIFOs
  with host-visible fill levels and a configurable `MOV STATUS` threshold
- Every instruction carries a 5-bit delay/side-set field → cycle-exact pin
  timing; side-set can be optional per instruction and can drive pin
  *directions* for open-drain protocols like I2C
- 16-entry GPIO space: 8 bidirectional (`uio`), 5 input-only (`ui[7:3]`),
  3 output-only (`uo[4:2]`); per-pin priority router, pins hold last driven value
- 4 shared IRQ flags for inter-SM sync and host interrupts (maskable `HOST_IRQ` pin)
- Host interface: mode-0 SPI slave (clk >= 8x SCK), command byte
  `{RW, ADDR[6:0]}`, auto-incrementing address, FIFO registers held

## ISA

Instruction format: `[15:13] opcode | [12:8] delay/side-set | [7:0] operands`

| Opcode | Instruction | Function |
|--------|-------------|----------|
| 000 | `JMP cond, addr` | always, !X, X--, !Y, Y--, X!=Y, PIN, !OSRE |
| 001 | `WAIT pol, src, idx` | stall on GPIO / relative pin / IRQ flag |
| 010 | `IN src, n` | shift 1-8 bits into ISR |
| 011 | `OUT dst, n` | shift 1-8 bits out of OSR |
| 100 | `PUSH` / `PULL` | ISR -> RX FIFO / TX FIFO -> OSR |
| 101 | `MOV dst, op(src)` | copy / invert / bit-reverse; `MOV EXEC` injects instructions |
| 110 | `IRQ set/clr/wait idx` | 4 shared flags |
| 111 | `SET dst, imm` | drive pins/pindirs, load X/Y |

A Python assembler for the ISA lives in
[sw/metastable_asm.py](sw/metastable_asm.py); it runs on CPython and
MicroPython and is shared by the cocotb suite and the demo-board host
driver ([sw/metastable_host.py](sw/metastable_host.py)).
[sw/examples/uart_loopback.py](sw/examples/uart_loopback.py) runs the
verified UART demo on real silicon from the TT demo board's RP2040.

## Testing

```sh
cd test && make
```

Verification has three legs: a directed cocotb suite with eight protocol
demos and ISA corner coverage, constrained-random co-simulation against a
golden architectural model, and formal proofs.

The cocotb suite ([test/test.py](test/test.py)) covers:

- **UART loopback** — SM0 transmits 8N1 frames on GPIO0, SM1 receives them
  on the same pin through the pin router
- **SPI master/slave** — SM0 clocks a mode-0 master via optional side-set,
  SM1 answers full duplex with autopull/autopush
- **I2C master** — open-drain address + data write (side-set on pin
  directions) against a Python slave model that ACKs each byte
- **Manchester loopback** — SM0 transmits IEEE 802.3 Manchester with a
  mid-bit transition every 8 ticks, SM1 locks onto the preamble and decodes
- **WS2812 / NeoPixel** — SM0 drives 800 kHz pulse-width-coded LED data
  (fractional divider at 8 MHz tick); a decoder model checks every pulse
  against the datasheet windows
- **USB low-speed TX** — a real DATA0 packet (sync, NRZI, bit stuffing,
  CRC16, EOP) on D+/D- at 1.5 MHz +0.03% via the fractional divider; an
  independent decoder model samples the bus at bit centers and checks
  every field plus the measured bit rate
- **PS/2 device** — device-generated 12.5 kHz clock via side-set, 11-bit
  frames (start, data, odd parity, stop) spanning two FIFO bytes with
  mid-frame autopull; host model validates framing and the clock band
- **Concurrent protocols** — SM0 transmits the USB LS packet at 1.5 MHz
  while SM1 drives a WS2812 frame at 800 kHz, both decoders clean: two
  unrelated bit rates from one chip at once
- **NEC infrared receive** — decodes address + command by measuring gap
  length (WAIT for burst, WAIT for burst end, sample a fixed delay
  later): a time-measuring receiver, the skill IR/1-Wire/DHT protocols
  need
- **Capture / replay** — SM1 replays an arbitrary 4-bit waveform from
  its FIFO while SM0 samples the same pins into a self-freezing
  16-sample snapshot: one chip as both ends of a logic analyzer
- **ISA coverage** — MOV invert/reverse/STATUS/EXEC, inter-SM IRQ
  handshake + HOST_IRQ, JMP variants, computed jumps, FIFO thresholds and
  backpressure, fractional clock divider, restart semantics
- **Constrained-random cosim** — seeded random programs run against a
  golden Python model of the SM ([test/golden.py](test/golden.py)):
  one suite over the architectural subset (PC, IRQ flags, exact RX FIFO
  contents, stall-park points) and one over the pin datapath (pins,
  pindirs, optional/non-optional side-set decoded from the raw
  instruction word, JMP PIN, random pin-base configs across the 16-pin
  space, GPIO readback compared bit-exactly)
- **SPI host-interface fuzz** — random read/write bursts interleaved
  with transfers aborted by CS_N mid-byte at random bit offsets, checked
  against a reference model: only fully clocked bytes commit

Formal ([formal/run.sh](formal/run.sh), CI job `formal`, yosys-smtbmc
BMC + k-induction, properties under `ifdef FORMAL`) covers three blocks:
the FIFO's structural invariants and in-order data integrity; the clock
divider's timing theorem — every tick-to-tick gap is exactly
`div_int + carry`, so 256 ticks span exactly `256*div_int + div_frac`
cycles, zero cumulative drift; and the SPI slave's register-bus strobe
discipline (single-cycle, exclusive, exactly-once) proved with no
assumptions at all on the pad waveforms — arbitrary SCK glitches and
mid-byte aborts included.

## Physical design

[analysis/](analysis/README.md) audits the hardened layout with the proxy
lenses from [VivaPlace](https://github.com/SameerSul/leetfm-macro-place-challenge-2026)
(HPWL / density / RUDY congestion + connectivity-inferred hierarchy cohesion).
The audit traced the worst setup path to a die-spanning scatter of SM1's
cluster; a validated placement-density change recovered 4.3x slow-corner
setup margin (+0.34 to +1.45 ns at 50 MHz).

## Resources

- [Tiny Tapeout FAQ](https://tinytapeout.com/faq/)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)
- [RP2040 datasheet, ch. 3 (PIO)](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) — the inspiration
