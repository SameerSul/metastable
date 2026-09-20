"""VivaPlace-style floorplan analysis of the hardened Metastable macro.

Reuses VivaPlace's eda_io LEF/DEF parsers, then applies the same three
proxy lenses its placer optimizes (HPWL wirelength, density grid, RUDY
congestion) plus its core idea — hierarchy inference from netlist
connectivity — to audit how well OpenROAD's flat placement kept the RTL
modules (SM0, SM1, the four FIFOs, SPI slave) spatially coherent.

Run from the VivaPlace repo root:
    uv run python <this file> <def> <stdcell.lef> <outdir>
"""

import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.cwd()))
from src.eda_io.def_io import parse_def
from src.eda_io.lef import parse_lef

DEF_PATH, LEF_PATH, OUT = sys.argv[1], sys.argv[2], Path(sys.argv[3])
OUT.mkdir(parents=True, exist_ok=True)

design = parse_def(DEF_PATH)
# VivaPlace's LEF reader predates PROPERTYDEFINITIONS blocks (the CMOS5L LEF
# defines a "MACRO CatenaDesignType" property that confuses it) — strip them.
lef_text = re.sub(r"PROPERTYDEFINITIONS.*?END PROPERTYDEFINITIONS", "",
                  Path(LEF_PATH).read_text(), flags=re.S)
clean_lef = OUT / "_clean.lef"
OUT.mkdir(parents=True, exist_ok=True)
clean_lef.write_text(lef_text)
parse_lef(clean_lef, design.masters)
x0, y0, x1, y1 = design.die_area
DIE_W, DIE_H = x1 - x0, y1 - y0

NONFUNC = re.compile(r"fill|decap|antenna|tap", re.I)


def is_func(comp):
    if NONFUNC.search(comp.master):
        return False
    return not re.match(r"^(FILLER|TAP|ANTENNA|PHY)", comp.name)


func = {n: c for n, c in design.components.items() if is_func(c) and c.pos}


def center(comp):
    m = design.masters.get(comp.master)
    if m is None:
        return comp.pos
    w, h = design.size_of(comp)
    return comp.pos[0] + w / 2, comp.pos[1] + h / 2


# ---------------------------------------------------------------- hierarchy
# Seed: named nets keep RTL register names; the FF driving pin Q gets the
# module label. Then propagate by majority vote over net neighbours
# (VivaPlace's hierarchy inference from connectivity, small-scale).
def label_of(net_name):
    n = net_name.replace("\\", "")
    m = re.match(r"(fifos\[\d\]\.u_[rt]x|sms\[\d\]|u_spi)\.", n)
    if m:
        return (
            m.group(1)
            .replace("fifos[", "fifo")
            .replace("].u_", "_")
            .replace("sms[", "sm")
            .replace("]", "")
            .replace("u_spi", "spi")
        )
    if not re.match(r"^_\d+_$", n) and "." not in n:
        return "top"
    return None


labels = {}
for net in design.nets:
    lab = label_of(net.name)
    if lab is None:
        continue
    for comp_name, pin in net.terms:
        if pin == "Q" and comp_name in func:
            labels[comp_name] = lab

seed_count = len(labels)

adj = defaultdict(set)
for net in design.nets:
    members = [c for c, p in net.terms if c in func]
    if len(members) > 32:  # skip huge nets (clock/reset) — no locality info
        continue
    for a in members:
        for b in members:
            if a != b:
                adj[a].add(b)

for _ in range(6):
    new = {}
    for name in func:
        if name in labels:
            continue
        votes = Counter(labels[nb] for nb in adj[name] if nb in labels)
        if votes:
            new[name] = votes.most_common(1)[0][0]
    if not new:
        break
    labels.update(new)

unlabeled = [n for n in func if n not in labels]

# per-module stats
mods = {}
for name, comp in func.items():
    lab = labels.get(name, "unassigned")
    w, h = design.size_of(comp)
    cx, cy = center(comp)
    mods.setdefault(lab, []).append((cx, cy, w * h))

mod_stats = {}
for lab, cells in mods.items():
    pts = np.array([(c[0], c[1]) for c in cells])
    area = sum(c[2] for c in cells)
    centroid = pts.mean(axis=0)
    spread = float(np.sqrt(((pts - centroid) ** 2).sum(axis=1).mean()))
    # ideal spread: radius of gyration of a square holding `area` at 70% util
    ideal = float(np.sqrt((area / 0.7) / 6))  # RMS radius of square side s: s/sqrt(6)
    bbox = (pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max())
    mod_stats[lab] = dict(
        cells=len(cells),
        area_um2=round(area, 1),
        centroid=[round(float(v), 1) for v in centroid],
        rms_spread_um=round(spread, 2),
        ideal_rms_um=round(ideal, 2),
        cohesion=round(ideal / spread, 3) if spread > 0 else 1.0,
        bbox=[round(float(v), 1) for v in bbox],
    )

# ---------------------------------------------------------------- proxies
BX, BY = 43, 24  # ~30 µm bins
bw, bh = DIE_W / BX, DIE_H / BY
density = np.zeros((BY, BX))
for name, comp in func.items():
    w, h = design.size_of(comp)
    cx, cy = comp.pos
    ix0, ix1 = int((cx - x0) / bw), int(min((cx + w - x0) / bw, BX - 1e-9))
    iy0, iy1 = int((cy - y0) / bh), int(min((cy + h - y0) / bh, BY - 1e-9))
    for iy in range(iy0, iy1 + 1):
        for ix in range(ix0, ix1 + 1):
            ox = min(cx + w, x0 + (ix + 1) * bw) - max(cx, x0 + ix * bw)
            oy = min(cy + h, y0 + (iy + 1) * bh) - max(cy, y0 + iy * bh)
            density[iy, ix] += max(ox, 0) * max(oy, 0)
density /= bw * bh

rudy = np.zeros((BY, BX))
hpwl = 0.0
for net in design.nets:
    pts = [center(func[c]) for c, p in net.terms if c in func]
    pts += [design.io_pins[c2].pos for c2, p in net.terms if p == "PIN"
            for c2 in [c2] if c2 in design.io_pins and design.io_pins[c2].pos]
    # DEF stores ("PIN", name)? normalise below if needed
    for c, p in net.terms:
        if c == "PIN" and p in design.io_pins and design.io_pins[p].pos:
            pts.append(design.io_pins[p].pos)
    if len(pts) < 2:
        continue
    xs, ys = zip(*pts)
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    hpwl += w + h
    if w * h == 0:
        continue
    dem = (w + h) / (w * h)
    jx0, jx1 = int((min(xs) - x0) / bw), int(min((max(xs) - x0) / bw, BX - 1e-9))
    jy0, jy1 = int((min(ys) - y0) / bh), int(min((max(ys) - y0) / bh, BY - 1e-9))
    for iy in range(jy0, jy1 + 1):
        for ix in range(jx0, jx1 + 1):
            ox = min(max(xs), x0 + (ix + 1) * bw) - max(min(xs), x0 + ix * bw)
            oy = min(max(ys), y0 + (iy + 1) * bh) - max(min(ys), y0 + iy * bh)
            rudy[iy, ix] += dem * max(ox, 0) * max(oy, 0) / (bw * bh)

summary = dict(
    die_um=[round(DIE_W, 1), round(DIE_H, 1)],
    functional_cells=len(func),
    total_components=len(design.components),
    nets=len(design.nets),
    hpwl_um=round(hpwl, 0),
    seed_ffs=seed_count,
    labeled=len(func) - len(unlabeled),
    unlabeled=len(unlabeled),
    density_max=round(float(density.max()), 3),
    density_mean=round(float(density.mean()), 3),
    rudy_max=round(float(rudy.max()), 3),
    rudy_mean=round(float(rudy.mean()), 3),
    modules=mod_stats,
)
(OUT / "floorplan_report.json").write_text(json.dumps(summary, indent=2))
print(json.dumps({k: v for k, v in summary.items() if k != "modules"}, indent=2))
for lab in sorted(mod_stats):
    s = mod_stats[lab]
    print(f"{lab:12s} cells={s['cells']:5d} area={s['area_um2']:9.0f} "
          f"spread={s['rms_spread_um']:6.1f} ideal={s['ideal_rms_um']:6.1f} "
          f"cohesion={s['cohesion']:.3f} centroid={s['centroid']}")

# ---------------------------------------------------------------- plot
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

palette = {
    "sm0": "#1f77b4", "sm1": "#ff7f0e",
    "fifo0_tx": "#2ca02c", "fifo0_rx": "#98df8a",
    "fifo1_tx": "#d62728", "fifo1_rx": "#ff9896",
    "spi": "#9467bd", "top": "#8c564b", "unassigned": "#cccccc",
}
fig, axes = plt.subplots(1, 3, figsize=(19, 4.4))
ax = axes[0]
for name, comp in func.items():
    w, h = design.size_of(comp)
    ax.add_patch(Rectangle(comp.pos, w, h,
                 color=palette.get(labels.get(name, "unassigned"), "#cccccc"),
                 lw=0))
ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal")
ax.set_title("Placement by inferred RTL module")
handles = [plt.Line2D([], [], marker="s", ls="", color=c, label=l)
           for l, c in palette.items()]
ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.0, 0.5),
          fontsize=7, frameon=False)

for ax, grid, title in ((axes[1], density, "Cell density / bin"),
                        (axes[2], rudy, "RUDY congestion estimate")):
    im = ax.imshow(grid, origin="lower", extent=(x0, x1, y0, y1),
                   cmap="inferno", aspect="equal")
    ax.set_title(title)
    fig.colorbar(im, ax=ax, fraction=0.03)
fig.suptitle("Metastable (tt_um_sulems6_metastable) — VivaPlace-lens floorplan audit", y=1.02)
fig.tight_layout()
fig.savefig(OUT / "floorplan_audit.png", dpi=160, bbox_inches="tight")
print("wrote", OUT / "floorplan_audit.png")
