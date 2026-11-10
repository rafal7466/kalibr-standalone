"""Summary figure for the README: kalibr-standalone accuracy vs. the official EuRoC calibration.

Numbers are taken from results/report_cam_april__imu_april.md (cam_april + imu_april runs).
Usage:  python scripts/make_readme_figure.py [output_dir]   (default: docs/images)
Writes accuracy_vs_euroc_light.png and accuracy_vs_euroc_dark.png."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

THEMES = {
    "light": dict(surface="#fcfcfb", text="#0b0b0b", text2="#52514e", grid="#e4e3df",
                  axis="#a8a7a2", s1="#2a78d6", s2="#eb6834"),
    "dark":  dict(surface="#1a1a19", text="#f0efec", text2="#c3c2b7", grid="#33322f",
                  axis="#5d5c58", s1="#3987e5", s2="#d95926"),
}

# --- data -------------------------------------------------------------------
params = ["f$_u$", "f$_v$", "c$_u$", "c$_v$"]
cam0 = [0.107, 0.093, -0.685, 0.424]     # kalibr-standalone minus EuRoC [px]
cam1 = [0.113, 0.101, -1.464, 0.491]
transforms = ["cam0 → cam1\n(stereo)", "cam0 ← IMU", "cam1 ← IMU"]
rot_err = [0.087, 0.279, 0.364]           # [deg]
trans_err = [0.47, 7.70, 8.00]            # [mm]


def rounded_bars(ax, xs, heights, width, color, surface, radius_px=4):
    """One shape per bar: square at the zero baseline, ~4px rounded corners at the data end.
    Drawn in display coordinates, so call after the layout is final."""
    from matplotlib.path import Path
    from matplotlib.patches import PathPatch
    from matplotlib.transforms import IdentityTransform
    fig = ax.figure
    r = radius_px * fig.dpi / 72.0
    gap = 1.0 * fig.dpi / 72.0          # ~2px surface gap between neighbouring bars
    for x, h in zip(xs, heights):
        (xa, ya), (xb, yb) = ax.transData.transform([(x - width / 2, 0), (x + width / 2, h)])
        xa, xb = xa + gap / 2, xb - gap / 2
        up = yb >= ya
        rr = min(r, abs(yb - ya) / 2, (xb - xa) / 2)
        s = 1 if up else -1
        verts = [(xa, ya), (xa, yb - s * rr), (xa, yb), (xa + rr, yb),
                 (xb - rr, yb), (xb, yb), (xb, yb - s * rr), (xb, ya), (xa, ya)]
        codes = [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3,
                 Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY]
        ax.add_artist(PathPatch(Path(verts, codes), transform=IdentityTransform(),
                                facecolor=color, edgecolor="none", clip_on=False, zorder=2))


def style_axes(ax, t):
    ax.set_facecolor(t["surface"])
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(t["axis"])
    ax.tick_params(colors=t["text2"], labelsize=9, length=0)
    ax.yaxis.grid(True, color=t["grid"], linewidth=0.8)
    ax.set_axisbelow(True)


def make(theme, out):
    t = THEMES[theme]
    plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": t["text"]})
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.0), dpi=150,
                             gridspec_kw=dict(width_ratios=[1.35, 1, 1]))
    fig.patch.set_facecolor(t["surface"])
    pending = []   # bars are drawn in display coords, after the layout is final

    # Panel 1: intrinsics difference (grouped, 2 series)
    ax = axes[0]
    style_axes(ax, t)
    ax.set_ylim(-1.7, 0.75)
    w = 0.36
    xs = range(len(params))
    pending.append((ax, [x - w / 2 for x in xs], cam0, w, t["s1"]))
    pending.append((ax, [x + w / 2 for x in xs], cam1, w, t["s2"]))
    ax.axhline(0, color=t["axis"], linewidth=1)
    ax.set_xlim(-0.6, len(params) - 0.4)
    ax.set_xticks(list(xs))
    ax.set_xticklabels(params, fontsize=11, color=t["text"])
    ax.set_ylabel("difference to EuRoC [px]", color=t["text2"], fontsize=9)
    ax.set_title("Camera intrinsics", loc="left", fontsize=12, fontweight="bold", color=t["text"])
    for x, v in zip(xs, cam1):     # label only the largest deviation, selectively
        if abs(v) > 1:
            ax.annotate(f"{v:+.2f} px", (x + w, v), xytext=(4, 2), textcoords="offset points",
                        ha="left", va="center", fontsize=8.5, color=t["text2"])
    ax.annotate("focal lengths: +0.1 px (0.02 %)", (0.5, 0.42), xytext=(0, 14),
                textcoords="offset points", ha="center", fontsize=8.5, color=t["text2"])
    # legend (2 series) with swatches; text in text color
    from matplotlib.patches import Patch
    leg = ax.legend(handles=[Patch(color=t["s1"], label="cam0"), Patch(color=t["s2"], label="cam1")],
                    loc="lower left", frameon=False, fontsize=9, handlelength=1.0, handleheight=1.0)
    for txt in leg.get_texts():
        txt.set_color(t["text"])

    # Panel 2 and 3: extrinsic errors (single series each -> no legend)
    for ax, vals, labels, title, top in (
            (axes[1], rot_err, ["0.087°", "0.28°", "0.36°"], "Rotation error", 0.45),
            (axes[2], trans_err, ["0.47 mm", "7.7 mm", "8.0 mm"], "Translation error", 10.0)):
        style_axes(ax, t)
        ax.set_ylim(0, top)
        xs = range(len(transforms))
        pending.append((ax, list(xs), vals, 0.55, t["s1"]))
        ax.axhline(0, color=t["axis"], linewidth=1)
        ax.set_xlim(-0.6, len(transforms) - 0.4)
        ax.set_xticks(list(xs))
        ax.set_xticklabels(transforms, fontsize=8.5, color=t["text"])
        ax.set_title(title, loc="left", fontsize=12, fontweight="bold", color=t["text"])
        for x, v, lab in zip(xs, vals, labels):
            ax.annotate(lab, (x, v), xytext=(0, 4), textcoords="offset points",
                        ha="center", fontsize=9, color=t["text"])
    axes[2].set_ylabel("vs. EuRoC reference [mm]", color=t["text2"], fontsize=9)
    axes[1].set_ylabel("vs. EuRoC reference [deg]", color=t["text2"], fontsize=9)

    fig.suptitle("kalibr-standalone vs. official EuRoC calibration  (cam_april: 1450 stereo pairs, imu_april: 200 Hz IMU)",
                 x=0.012, ha="left", fontsize=10, color=t["text2"], y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.95), w_pad=2.5)
    fig.canvas.draw()
    for ax, xs, vals, w, color in pending:
        rounded_bars(ax, xs, vals, w, color, t["surface"])
    fig.savefig(out, facecolor=t["surface"])
    plt.close(fig)


if __name__ == "__main__":
    outdir = sys.argv[1] if len(sys.argv) > 1 else "docs/images"
    for th in THEMES:
        make(th, f"{outdir}/accuracy_vs_euroc_{th}.png")
