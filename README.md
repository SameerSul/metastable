![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Metastable: programmable protocol emulator

A general-purpose protocol emulator ASIC for the Jane Street protocol emulator
competition, built on [Tiny Tapeout](https://tinytapeout.com) (IHP 130nm CMOS5L,
6x4 tiles). Two RP2040-PIO-inspired programmable state machines bit-bang
arbitrary serial protocols — UART, SPI, I2C, and anything else that fits the
timing — entirely in firmware loaded over SPI after fabrication.

Full datasheet: [docs/info.md](docs/info.md)

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
[test/metastable.py](test/metastable.py), along with an SPI host driver.

## Testing

```sh
cd test && make
```

Runs the cocotb suite ([test/test.py](test/test.py)): register/instruction
memory read-write, three protocol demos, and ISA corner coverage:

- **UART loopback** — SM0 transmits 8N1 frames on GPIO0, SM1 receives them
  on the same pin through the pin router
- **SPI master/slave** — SM0 clocks a mode-0 master via optional side-set,
  SM1 answers full duplex with autopull/autopush
- **I2C master** — open-drain address + data write (side-set on pin
  directions) against a Python slave model that ACKs each byte
- **ISA coverage** — MOV invert/reverse/STATUS/EXEC, inter-SM IRQ
  handshake + HOST_IRQ, JMP variants, computed jumps, FIFO thresholds and
  backpressure, fractional clock divider, restart semantics

## Resources

- [Tiny Tapeout FAQ](https://tinytapeout.com/faq/)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)
- [RP2040 datasheet, ch. 3 (PIO)](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) — the inspiration
