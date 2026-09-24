"""Architecture block diagram for the Metastable datasheet."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

INK = "#1a2030"
ACCENT = "#0e7490"
FILL = "#eef2f7"
FILL2 = "#e0f2f7"

fig, ax = plt.subplots(figsize=(11.2, 6.0))
ax.set_xlim(0, 112)
ax.set_ylim(0, 65)
ax.axis("off")


def box(x, y, w, h, label, sub=None, fill=FILL, lw=1.4, fs=10.5, bold=True):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                 boxstyle="round,pad=0.35,rounding_size=1.1",
                 fc=fill, ec=INK, lw=lw))
    cy = y + h / 2 + (1.6 if sub else 0)
    ax.text(x + w / 2, cy, label, ha="center", va="center", fontsize=fs,
            color=INK, fontweight="bold" if bold else "normal")
    if sub:
        ax.text(x + w / 2, y + h / 2 - 2.2, sub, ha="center", va="center",
                fontsize=8.2, color="#47526b")


def arrow(x0, y0, x1, y1, both=False, color=INK, lw=1.5):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1),
                 arrowstyle="<|-|>" if both else "-|>",
                 mutation_scale=13, color=color, lw=lw,
                 shrinkA=0, shrinkB=0))


def sm(x, y, name):
    box(x, y, 30, 26, "", fill="white")
    ax.text(x + 15, y + 23.2, name, ha="center", fontsize=11.5,
            fontweight="bold", color=ACCENT)
    box(x + 1.6, y + 13.5, 12.8, 6.5, "CLKDIV", "16.8 frac", fill=FILL2, fs=8.6)
    box(x + 15.6, y + 13.5, 12.8, 6.5, "EXEC", "decode+delay", fill=FILL2, fs=8.6)
    box(x + 1.6, y + 6.6, 12.8, 5.6, "OSR / ISR", "8b shifters", fill=FILL2, fs=8.2)
    box(x + 15.6, y + 6.6, 12.8, 5.6, "X / Y", "scratch", fill=FILL2, fs=8.2)
    box(x + 1.6, y + 0.6, 12.8, 4.6, "TX FIFO", "8 deep", fill=FILL2, fs=8.2)
    box(x + 15.6, y + 0.6, 12.8, 4.6, "RX FIFO", "8 deep", fill=FILL2, fs=8.2)


# host side
box(2, 24, 13, 13, "SPI\nslave", "mode 0\nclk >= 8x SCK", fs=10)
ax.text(8.5, 40.5, "HOST", ha="center", fontsize=9, color="#47526b",
        fontweight="bold")
ax.text(0.5, 33.5, "SCK\nCS_N\nMOSI\nMISO", ha="right", va="center",
        fontsize=7.6, color="#47526b", linespacing=1.5)
arrow(-0.2, 30.5, 2, 30.5, both=True)

# register file / imem
box(20, 38, 22, 14, "IMEM", "32 x 16-bit\nshared", fs=11)
box(20, 8, 22, 24, "REGS", "CTRL / FSTAT / IRQ\nclkdiv, wrap, shift,\npins, FIFO access,\nfill levels, STATUS", fs=11)

# state machines
sm(48, 32, "SM0")
sm(48, 2, "SM1")

# IRQ block
box(83, 27.5, 10, 7, "IRQ", "4 flags", fill=FILL2, fs=9)

# pin router
box(96, 8, 12, 44, "PIN\nROUTER", "per-pin\npriority,\nhold last", fs=10.5)
ax.text(110.8, 44, "GPIO0-7\nbidir", ha="left", va="center", fontsize=7.8,
        color="#47526b", linespacing=1.4)
ax.text(110.8, 30, "GPIO8-12\nin only", ha="left", va="center", fontsize=7.8,
        color="#47526b", linespacing=1.4)
ax.text(110.8, 16, "GPIO13-15\nout only", ha="left", va="center", fontsize=7.8,
        color="#47526b", linespacing=1.4)
for yy in (44, 30, 16):
    arrow(108.4, yy, 110.6, yy, both=(yy == 44))

# wiring
arrow(15.4, 30.5, 19.6, 30.5, both=True)          # spi <-> regs
arrow(15.4, 33.5, 19.6, 44, both=False)           # spi -> imem (program load)
arrow(42.4, 45, 48, 45, both=False)               # imem -> sm0
arrow(42.4, 43, 46, 43, both=False)
arrow(46, 43, 46, 15, both=False)
arrow(46, 15, 48, 15, both=False)                 # imem -> sm1
arrow(42.4, 24, 48, 24, both=True)                # regs <-> sm0 (fifo/cfg)
arrow(42.4, 18, 48, 18, both=True)                # regs <-> sm1
arrow(78.4, 45, 96, 45, both=True)                # sm0 <-> router
arrow(78.4, 15, 96, 15, both=True)                # sm1 <-> router
arrow(78.4, 33, 82.6, 32, both=True)              # sm0 <-> irq
arrow(78.4, 26, 82.6, 29, both=True)              # sm1 <-> irq
arrow(88, 27.1, 88, 22, both=False)               # irq -> host_irq? route down
ax.text(88.6, 24.5, "HOST_IRQ", fontsize=7.6, color="#47526b", ha="left")

ax.text(56, 62.5, "Metastable: programmable protocol emulator",
        ha="center", fontsize=14, fontweight="bold", color=INK)

fig.tight_layout()
fig.savefig("/Users/ssuleman/Desktop/ML4PD Projects/Metastable/metastable/docs/architecture.png",
            dpi=150, bbox_inches="tight", facecolor="white")
print("saved")
