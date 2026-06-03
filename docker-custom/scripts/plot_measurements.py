#!/usr/bin/env python3
"""Plot the three physical sensor measurements in their REAL units (no scaling):

  1. LiDAR : distance from the front wall (m)
  2. Camera: checkerboard apparent width (px) -- horizontal span of inner corners
  3. IMU   : raw linear accelerations ax, ay, az (m/s^2)

Reads bags/<name>_signals.npz (from extract_signals.py). Each series is plotted
against time since its OWN first stamp (t0), because the camera (host clock) and
the LiDAR/IMU (AIRY hardware clock) live on different epochs; the absolute t0 of
each is printed so the epoch gap is explicit.

Usage:
  python3 plot_measurements.py <bag_name> [--show] [--out PATH]
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("TkAgg" if "--show" in sys.argv else "Agg")
import matplotlib.pyplot as plt

BAGS_DIR = "/opt/custom/bags"


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

    lidar_t, cam_t, imu_t = d["lidar_t"], d["cam_t"], d["imu_t"]
    cam_epoch = float(d["cam_epoch"]) if "cam_epoch" in d.files else 0.0

    # All stamps are already on the common (lidar) clock -- the camera epoch was
    # removed at extraction. Use ONE shared reference so the three measurements
    # sit on the same timeline.
    starts = [float(a[0]) for a in (lidar_t, cam_t, imu_t) if len(a)]
    t_ref = min(starts) if starts else 0.0
    print(f"common clock = AIRY lidar clock; camera epoch removed = {cam_epoch:.6f} s")
    print(f"shared time reference t_ref = {t_ref:.6f} s")

    fig, (a0, a1, a2) = plt.subplots(3, 1, sharex=True, figsize=(12, 9))

    a0.plot(lidar_t - t_ref, d["lidar_dist"], color="tab:blue")
    a0.set_ylabel("distance to wall (m)")
    a0.set_title(f"sensor measurements: {base}  (common lidar clock)")

    a1.plot(cam_t - t_ref, d["cam_width"], color="tab:green")
    a1.set_ylabel("checkerboard width (px)")

    a2.plot(imu_t - t_ref, d["imu_ax"], label="ax", lw=0.9)
    a2.plot(imu_t - t_ref, d["imu_ay"], label="ay", lw=0.9)
    a2.plot(imu_t - t_ref, d["imu_az"], label="az", lw=0.9)
    a2.set_ylabel("acceleration (m/s$^2$)")
    a2.legend(loc="upper right", ncol=3, fontsize=8)
    a2.set_xlabel("time on common lidar clock, since t_ref (s)")

    for ax in (a0, a1, a2):
        ax.grid(True, alpha=0.3)
    fig.tight_layout()

    out = args.out or os.path.join(BAGS_DIR, base + "_measurements.png")
    fig.savefig(out, dpi=120)
    print(f"\nranges:")
    print(f"  wall distance : {np.nanmin(d['lidar_dist']):.3f} .. {np.nanmax(d['lidar_dist']):.3f} m")
    print(f"  board width   : {np.nanmin(d['cam_width']):.1f} .. {np.nanmax(d['cam_width']):.1f} px")
    print(f"-> {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
