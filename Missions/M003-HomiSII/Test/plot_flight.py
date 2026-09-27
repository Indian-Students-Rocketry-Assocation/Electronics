"""Plot altitude, velocity and vertical acceleration from a flight computer log.

Usage: python3 plot_flight.py [flight.csv] [out.png]
"""
import csv
import sys

import matplotlib.pyplot as plt

SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
SERIES = "#2a78d6"

src = sys.argv[1] if len(sys.argv) > 1 else "flight.csv"
out = sys.argv[2] if len(sys.argv) > 2 else "flight.png"

rows = list(csv.DictReader(open(src)))
t0 = float(rows[0]["ms"])
t = [(float(r["ms"]) - t0) / 1000 for r in rows]
col = lambda k: [float(r[k]) for r in rows]

plt.rcParams.update({
    "font.size": 10, "text.color": INK, "axes.labelcolor": INK_2,
    "xtick.color": INK_2, "ytick.color": INK_2, "axes.edgecolor": GRID,
})
fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE)

panels = [
    ("Altitude (m)", "h_m", "h_baro_m"),
    ("Vertical velocity (m/s)", "v_ms", None),
    ("Vertical acceleration (m/s²)", "a_vert_ms2", None),
]
for ax, (label, key, raw) in zip(axs, panels):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.axhline(0, color=INK_2, linewidth=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    if raw:
        ax.plot(t, col(raw), color=INK_2, linewidth=1, alpha=0.45, label="Barometer (raw)")
    ax.plot(t, col(key), color=SERIES, linewidth=2, label="Kalman filter" if raw else None)
    ax.set_ylabel(label)

    # shade any non-SAFE states (calibration, flight) so they are easy to find
    i = 0
    while i < len(rows):
        s = rows[i]["state"]
        j = i
        while j < len(rows) and rows[j]["state"] == s:
            j += 1
        if s != "SAFE":
            ax.axvspan(t[i], t[j - 1], color=GRID, alpha=0.6, linewidth=0)
            if ax is axs[0]:
                ax.text(t[i], ax.get_ylim()[1], f" {s}", va="top", fontsize=8, color=INK_2)
        i = j

axs[0].legend(frameon=False, loc="lower left")
axs[-1].set_xlabel("Time since log start (s)")
fig.suptitle(f"{src}", x=0.06, ha="left", fontsize=12)
fig.tight_layout()
fig.savefig(out, dpi=150, facecolor=SURFACE)
print(out)
