# Host software

Pure-Python tools for driving the chip, shared verbatim with the cocotb
testbench — the assembler the tests verify is the assembler you ship.
Everything runs on CPython and MicroPython (the TT demo board's RP2040).

| File | Role |
|------|------|
| `metastable_asm.py` | ISA assembler, register map constants, and `sm_config()`: every configuration register of one state machine as a list of writes |
| `metastable_host.py` | register-level driver over bit-banged mode-0 SPI |
| `examples/uart_loopback.py` | SM0 transmits 8N1 on GPIO0, SM1 receives, bytes round-trip over SPI |
| `examples/ws2812.py` | 800 kHz NeoPixel driver on GPIO0 |
| `examples/i2c_write.py` | open-drain I2C master write, ACK bits back to the host |
| `examples/i2c_read.py` | open-drain I2C master read at 100 kHz or 400 kHz, with clock stretching and a STOP when no device answers |
| `examples/spi_modes.py` | SPI master and slave programs for modes 0 to 3, looped through the chip's own pads |
| `examples/capture_replay.py` | 4-channel snapshot logic analyzer + arbitrary waveform generator on GPIO4-7 |
| `examples/manchester.py` | IEEE 802.3 Manchester: SM0 transmits on GPIO0, SM1 locks onto the preamble and decodes |
| `examples/usb_ls_tx.py` | USB low-speed transmit at 1.5 Mb/s, with the host-side packet builder (sync, NRZI, bit stuffing, CRC16, EOP); needs a host that paces its SPI writes |
| `examples/ps2_device.py` | PS/2 device: 11-bit frames with a device-generated 12.5 kHz clock |
| `examples/nec_ir_receive.py` | NEC infrared receive on GPIO8 by gap-length measurement |
| `examples/two_protocols.py` | USB low-speed on SM0 and WS2812 on SM1 at the same time |

Every protocol in the cocotb suite has its example file here, and a test
that loads the program and the register settings from that file. So the
suite checks what ships.

## How the host side is tested

`test/test_examples.py` runs each example the way a user does: it calls
the example's own `main()` or host function. In that file the name
`ttboard.demoboard` is a stand-in (`test/standin`), with the simulated
chip behind the board's pins. So the driver in `metastable_host.py`, the
example's host code and the program run together, on the RTL and on the
gate-level netlist.

The stand-in's host is slow on purpose, as a bit-banged host is. Four
examples therefore load their data while the state machine is stopped,
and start it afterwards: `ws2812.py`, `manchester.py`, `ps2_device.py`
and `capture_replay.py`. One call of these takes what the 8-byte FIFO
holds, for example two LEDs. `usb_ls_tx.py` and `two_protocols.py` need
a host that paces its SPI writes. They have no host function, and so no
test of this kind.

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
for addr, value in config():          # divider, wrap region, pin mapping:
    host.write(addr, value)           #   every register of the state machine
host.ctrl(0b01, 0b01)                 # enable + restart SM0
host.tx(0, data)                      # feed the TX FIFO
host.rx(0, n)                         # drain the RX FIFO
```

An example's `config()` is built with `sm_config()`, which writes every
configuration register and leaves the ones it does not name at their
reset values. An example therefore does not depend on what ran before it.

See the register map in [docs/info.md](../docs/info.md) for every field.
