# Host software

Pure-Python tools for driving the chip, shared verbatim with the cocotb
testbench — the assembler the tests verify is the assembler you ship.
Everything runs on CPython and MicroPython (the TT demo board's RP2040).

| File | Role |
|------|------|
| `metastable_asm.py` | ISA assembler + register map constants |
| `metastable_host.py` | register-level driver over bit-banged mode-0 SPI |
| `examples/uart_loopback.py` | SM0 transmits 8N1 on GPIO0, SM1 receives, bytes round-trip over SPI |
| `examples/ws2812.py` | 800 kHz NeoPixel driver on GPIO0 |
| `examples/i2c_write.py` | open-drain I2C master write, ACK bits back to the host |
| `examples/capture_replay.py` | 4-channel snapshot logic analyzer + arbitrary waveform generator on GPIO4-7 |

## Demo board quick start

The chip's SPI slave sits on `ui[0]`=SCK, `ui[1]`=CS_N, `ui[2]`=MOSI,
`uo[0]`=MISO (any SPI master works; the driver bit-bangs, which trivially
satisfies the clk >= 8x SCK requirement). With the
[tt-micropython firmware](https://github.com/TinyTapeout/tt-micropython-firmware):

1. Copy `metastable_asm.py`, `metastable_host.py` and the examples onto
   the board.
2. `import uart_loopback; uart_loopback.main()`

The general flow every example follows:

```python
host = MetastableHost(drive, sample)  # two pin callables
host.load_program(0, PROG)            # assembler output -> imem
host.set_clkdiv(0, 6, 64)             # tick = clk / (6 + 64/256)
host.set_wrap(0, 0, 5)                # program region
host.write(SM(0) + PIN_SIDE, 0x10)    # pin mapping
host.ctrl(0b01, 0b01)                 # enable + restart SM0
host.tx(0, data)                      # feed the TX FIFO
host.rx(0, n)                         # drain the RX FIFO
```

See the register map in [docs/info.md](../docs/info.md) for every field.
