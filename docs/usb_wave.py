"""Render the USB LS packet waveform captured from simulation.

Reads test/usb_edges.json (dumped by test_usb_ls_tx) and draws D+/D- as
a scope-style trace: `cd test && make`, then run this from the repo root
with matplotlib available.
"""
import json
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

INK = "#1a2030"

here = pathlib.Path(__file__).resolve().parent
d = json.loads((here.parent / "test" / "usb_edges.json").read_text())
edges, t_bit = d["edges"], d["t_bit"]

start = next(t for t, s in edges if s == 0b01)  # first K of the sync
t0 = start - 4 * t_bit
end = max(t for t, _ in edges) + 6 * t_bit

xs, dp, dm = [t0], [0], [1]  # idle J before the packet
for t, s in edges:
    if t < t0:
        xs[0], dp[0], dm[0] = t0, s & 1, (s >> 1) & 1
        continue
    xs.append(t)
    dp.append(s & 1)
    dm.append((s >> 1) & 1)
xs.append(end)
dp.append(dp[-1])
dm.append(dm[-1])

fig, ax = plt.subplots(figsize=(11, 2.6))
us = [(x - start) / 1000 for x in xs]
ax.step(us, [v + 1.6 for v in dp], where="post", color="#0e7490", lw=1.6)
ax.step(us, dm, where="post", color="#b45309", lw=1.6)
ax.text(us[0] - 0.4, 2.1, "D+", ha="right", fontsize=11, color="#0e7490",
        fontweight="bold")
ax.text(us[0] - 0.4, 0.5, "D-", ha="right", fontsize=11, color="#b45309",
        fontweight="bold")

# annotate regions: sync is 8 bits from `start`, EOP is the SE0 pair
eop = next(t for t, s in edges if t > start and s == 0)
for x0, x1, label in [(0, 8 * t_bit / 1000, "SYNC"),
                      ((eop - start) / 1000, (eop - start) / 1000 + 2 * t_bit / 1000, "EOP")]:
    ax.axvspan(x0, x1, color="#0e7490", alpha=0.08)
    ax.text((x0 + x1) / 2, 2.95, label, ha="center", fontsize=8.5,
            color="#47526b")
ax.text((8 * t_bit / 1000 + (eop - start) / 1000) / 2, 2.95,
        "PID + DATA + CRC16 (NRZI, bit-stuffed)", ha="center", fontsize=8.5,
        color="#47526b")

ax.set_xlabel("time (us)  -  1.5 MHz bit clock from the fractional divider",
              fontsize=9.5, color=INK)
ax.set_yticks([])
ax.set_ylim(-0.4, 3.3)
for s in ("top", "right", "left"):
    ax.spines[s].set_visible(False)
fig.tight_layout()
fig.savefig(here / "usb_ls_wave.png", dpi=150, bbox_inches="tight",
            facecolor="white")
print("saved", here / "usb_ls_wave.png")
