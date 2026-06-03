#!/usr/bin/env python3
"""Plot every candidate channel of each sensor so you can SEE which axis the
motion excites, then set the dominant one in sync.yaml.

Reads bags/<name>_signals.npz (from extract_signals.py) and draws one subplot
per sensor with all its channels overlaid (z-scored so they share a scale):

  LiDAR  : plane-inlier centroid cx, cy, cz (m)   + distance
  Camera : checkerboard u, v (px), scale (px)
  IMU    : linear acceleration ax, ay, az (m/s^2)

For each sensor it prints the channels ranked by variance and names the
dominant one -- copy that into sync.yaml (imu.axis, checkerboard.signal).

Usage:
  python3 inspect_axes.py <bag_name> [--show] [--out PATH]
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("TkAgg" if "--show" in sys.argv else "Agg")
import matplotlib.pyplot as plt

BAGS_DIR = "/opt/custom/bags"


def _z(v):
    """Zero-mean, unit-std for overlay; flat signals pass through at 0."""
    v = np.asarray(v, dtype=float)
    s = v.std()
    return (v - v.mean()) / s if s > 1e-12 else v - v.mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_name")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    base = os.path.basename(args.bag_name.rstrip("/"))
    npz = args.bag_name if args.bag_name.endswith(".npz") else \
        os.path.join(BAGS_DIR, base + "_signals.npz")
    if not os.path.isfile(npz):
        sys.exit(f"signals not found: {npz}  (run extract_signals.py first)")
    d = np.load(npz, allow_pickle=True)

    # (sensor, time, t0, [(channel-name, values), ...], unit)
    sensors = [
        ("LiDAR", d["lidar_t"], float(d["lidar_t0"]),
         [("cx", d["lidar_cx"]), ("cy", d["lidar_cy"]), ("cz", d["lidar_cz"]),
          ("dist", d["lidar_dist"])], "m"),
        ("Camera", d["cam_t"], float(d["cam_t0"]),
         [("u", d["cam_u"]), ("v", d["cam_v_px"]), ("scale", d["cam_scale"])], "px"),
        ("IMU", d["imu_t"], float(d["imu_t0"]),
         [("ax", d["imu_ax"]), ("ay", d["imu_ay"]), ("az", d["imu_az"])], "m/s^2"),
    ]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
    print("\nper-channel std (dominant = largest motion), each sensor:")
    for ax, (name, t, t0, chans, unit) in zip(axes, sensors):
        if len(t) == 0:
            ax.text(0.5, 0.5, f"no {name} data", ha="center", va="center",
                    transform=ax.transAxes, color="red")
            ax.set_ylabel(name)
            print(f"  {name:7s}: (none)")
            continue
        tr = t - t0
        ranked = sorted(chans, key=lambda kv: np.nanstd(kv[1]), reverse=True)
        for ch, v in chans:
            ax.plot(tr, _z(v), lw=1.0, label=ch)
        ax.set_ylabel(f"{name}\n(z-scored)")
        ax.legend(loc="upper right", ncol=len(chans), fontsize=8)
        ax.grid(True, alpha=0.3)
        stats = "  ".join(f"{ch}={np.nanstd(v):.3g}{unit}" for ch, v in ranked)
        print(f"  {name:7s}: {stats}   -> dominant: {ranked[0][0]}")
    axes[-1].set_xlabel("time since each series' own t0 (s)")
    axes[0].set_title(f"axis inspection: {base}  (z-scored channels)")
    fig.tight_layout()

    out = args.out or os.path.join(BAGS_DIR, base + "_axes.png")
    fig.savefig(out, dpi=120)
    print(f"\n-> {out}")
    print("Set the dominant channel in sync.yaml: imu.axis (x|y|z), "
          "checkerboard.signal (u|v|scale).")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
