#!/usr/bin/env python3
"""Plot the three 1D sensor signals and align their motion reversals.

Loads bags/<name>_signals.npz (from extract_signals.py) and draws three stacked
subplots sharing a time axis:

  1. LiDAR distance to wall (m)        -- position
  2. Camera checkerboard center (px)   -- position
  3. IMU linear acceleration (m/s^2)   -- 2nd derivative

Each series is zero-based by its OWN first stamp so the shapes overlay despite
the sensors being on different clocks (the absolute t0 of each is printed).
Reversals are detected as smoothed local extrema (peaks of the signal and of
its negation) and drawn as vertical markers; for position signals these are the
turning points, for IMU they are the accel extrema that (anti-phase) coincide
with the same physical reversal. An offset table reports the time gap between
the nearest matched reversals across sensors.

Usage:
  python3 plot_sync.py <bag_name> [--config ...] [--show] [--out PATH]
"""
import argparse
import os
import sys

import numpy as np
import yaml
import matplotlib
# Pick the backend BEFORE importing pyplot: GUI (TkAgg) only if --show is asked
# for, else headless Agg so it works over SSH / in CI. matplotlib's TkAgg uses
# CPU/X rendering, so it does NOT hit the NVIDIA EGL/ZINK path the camera did.
matplotlib.use("TkAgg" if "--show" in sys.argv else "Agg")
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

BAGS_DIR = "/opt/custom/bags"
DEFAULT_CFG = "/opt/custom/config/sync.yaml"


def _norm_mid(t, v):
    """Min-max normalize v to 0..1 using its range over the MIDDLE HALF of the
    scan (time 25%..75%), so the stationary start/end of the sweep don't skew the
    limits. Returns (normalized_full_signal, vmin, vmax) where vmin/vmax are the
    real (physical-unit) extremes used. Values outside the middle half may fall
    slightly outside 0..1, which is fine."""
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    span = t[-1] - t[0] if len(t) > 1 else 0.0
    m = (t >= t[0] + 0.25 * span) & (t <= t[0] + 0.75 * span)
    seg = v[m] if np.count_nonzero(m) >= 2 else v
    vmin, vmax = float(np.nanmin(seg)), float(np.nanmax(seg))
    rng = vmax - vmin
    nv = (v - vmin) / rng if rng > 1e-12 else np.zeros_like(v)
    return nv, vmin, vmax


def _win_samples(t, seconds):
    """Convert a time window (s) to a sample count using each series' own rate,
    so the 200 Hz IMU is smoothed far more than the 10 Hz LiDAR for the same
    physical window."""
    dt = np.median(np.diff(t)) if len(t) > 1 else 1.0
    return max(1, int(round(seconds / max(dt, 1e-6))))


def smooth(t, v, seconds):
    w = _win_samples(t, seconds)
    if w <= 1 or len(v) < w:
        return v
    k = np.ones(w) / w
    return np.convolve(v, k, mode="same")


def reversals(t, v, smooth_s, min_sep_s, prom_frac):
    """Times of smoothed local maxima AND minima (turning points). A peak must
    rise/fall by at least `prom_frac` of the signal's robust amplitude (p95-p5)
    to count -- this rejects the noise wiggles that otherwise make the noisy IMU
    accel report many spurious reversals."""
    if len(t) < 3:
        return np.array([])
    vs = smooth(t, v, smooth_s)
    amp = np.percentile(vs, 95) - np.percentile(vs, 5)
    prom = max(prom_frac * amp, 1e-9)
    distance = _win_samples(t, min_sep_s)
    pk_hi, _ = find_peaks(vs, distance=distance, prominence=prom)
    pk_lo, _ = find_peaks(-vs, distance=distance, prominence=prom)
    idx = np.sort(np.concatenate([pk_hi, pk_lo]).astype(int))
    return t[idx]


def _pair_one_to_one(ref_rev, other_rev, tol):
    """Pair reversals ONE-TO-ONE by mutual nearest neighbour: ref[i] and other[j]
    pair iff each is the other's closest reversal in time. This guarantees every
    reversal is used at most once (so n can't exceed the reversal count) and
    leaves spurious/extra reversals -- e.g. the noisy IMU's -- unpaired instead
    of letting them corrupt the offset. Valid because the real offset (sub-second)
    is far smaller than the spacing between turning points, so true counterparts
    are unambiguously the nearest. A final tol gate (|diff - median| <= tol) drops
    any residual mismatch. Returns (offset, std, n_paired, pairs)."""
    ref_rev = np.asarray(ref_rev, dtype=float)
    other_rev = np.asarray(other_rev, dtype=float)
    if len(ref_rev) == 0 or len(other_rev) == 0:
        return float("nan"), float("nan"), 0, []
    ni = np.array([int(np.argmin(np.abs(other_rev - r))) for r in ref_rev])  # ref -> nearest other
    nj = np.array([int(np.argmin(np.abs(ref_rev - o))) for o in other_rev])  # other -> nearest ref
    pairs = [(i, int(ni[i])) for i in range(len(ref_rev)) if nj[ni[i]] == i]  # mutual
    if not pairs:
        return float("nan"), float("nan"), 0, []
    diffs = np.array([other_rev[j] - ref_rev[i] for i, j in pairs])
    keep = np.abs(diffs - np.median(diffs)) <= tol
    diffs, pairs = diffs[keep], [p for p, k in zip(pairs, keep) if k]
    if len(diffs) == 0:
        return float("nan"), float("nan"), 0, []
    return float(np.median(diffs)), float(np.std(diffs)), int(len(diffs)), pairs


def match_offsets(ref_name, ref_rev, others, tol):
    """Per (other, ref): constant offset (other - ref) from a ONE-TO-ONE pairing
    of their reversals, plus consistency (std) and paired count (<= #reversals).
    `others` is [(name, reversals)]. Positive => 'other' is stamped LATER than ref
    for the same physical turning point."""
    rows = []
    for name, rev in others:
        off, std, n, _ = _pair_one_to_one(ref_rev, rev, tol)
        rows.append((name, ref_name, off, std, n))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_name")
    ap.add_argument("--config", default=DEFAULT_CFG)
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    ap.add_argument("--out", default=None, help="PNG path (default bags/<name>_sync.png)")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    a = cfg["analysis"]
    smooth_s = a["smooth_seconds"]
    min_sep = a["min_reversal_sep_s"]
    prom_frac = a["min_prominence_frac"]

    base = os.path.basename(args.bag_name.rstrip("/"))
    npz_path = args.bag_name if args.bag_name.endswith(".npz") else \
        os.path.join(BAGS_DIR, base + "_signals.npz")
    if not os.path.isfile(npz_path):
        sys.exit(f"signals not found: {npz_path}  (run extract_signals.py first)")
    d = np.load(npz_path, allow_pickle=True)

    series = [
        ("lidar",  d["lidar_t"], d["lidar_v"], float(d["lidar_t0"]),
         "LiDAR distance (m)"),
        ("camera", d["cam_t"], d["cam_v"], float(d["cam_t0"]),
         f"Checkerboard {str(d['cam_axis'])} (px)"),
        ("imu",    d["imu_t"], d["imu_v"], float(d["imu_t0"]),
         f"IMU accel '{str(d['imu_axis'])}' (m/s^2)"),
    ]

    # All series are on the common (lidar) clock; use ONE shared reference.
    starts = [t0 for _, t, _, t0, _ in series if len(t)]
    t_ref = min(starts) if starts else 0.0
    cam_epoch = float(d["cam_epoch"]) if "cam_epoch" in d.files else 0.0
    tol = min_sep
    colors = {"lidar": "tab:blue", "camera": "tab:green", "imu": "tab:red"}

    # Detect reversals for every sensor (used for both the offsets and align).
    revs = {name: (reversals(t, v, smooth_s, min_sep, prom_frac) if len(t)
                   else np.array([])) for name, t, v, _, _ in series}

    print(f"\ncommon lidar clock (camera epoch removed = {cam_epoch:.6f} s)")
    print("first-stamp (t0) / reversals per sensor, on common clock:")
    for name, t, v, t0, _ in series:
        print(f"  {name:7s}: t0 {t0:.6f} s   ({len(revs[name])} reversals)")

    # Offset table: ONE-TO-ONE reversal pairing (mutual nearest). 'offset' =
    # other - ref for the same turning point; 'std' = consistency; 'n' = paired
    # count (<= reversal count). All on the common clock, so these are the real
    # sub-second lags. Positive => 'other' is stamped LATER than 'ref'.
    print("\nreversal offsets on common clock  (offset = other - ref; std = consistency):")
    print(f"  {'pair':16s} {'offset [s]':>18s} {'std [s]':>10s} {'n':>4s}")
    rows = match_offsets("imu", revs["imu"],
                         [(n, revs[n]) for n in ("lidar", "camera")], tol)
    rows += match_offsets("lidar", revs["lidar"],
                          [("camera", revs["camera"])], tol)
    for other, ref, off, std, n in rows:
        print(f"  {other+'-'+ref:16s} {off:18.4f} {std:10.4f} {n:4d}")

    # Per-sensor offset relative to the LiDAR (the alignment reference): shift
    # each signal EARLIER by this to time-sync. lidar is the anchor (0).
    align = {"lidar": 0.0}
    for name in ("camera", "imu"):
        align[name] = _pair_one_to_one(revs["lidar"], revs[name], tol)[0]  # name - lidar

    # ---- two stacked subplots: (top) before sync, (bottom) after sync ----
    # Each signal is min-max normalized to 0..1 using its range over the MIDDLE
    # HALF of the scan, so the stationary start/end don't skew the limits. The
    # real min/max (physical units) are shown in the legend.
    fig, (ax_raw, ax_sync) = plt.subplots(2, 1, sharex=True, figsize=(12, 8))

    def plot_panel(ax, apply_align):
        for name, t, v, _, label in series:
            if len(t) == 0:
                continue
            unit = label.split("(")[-1].rstrip(")")
            vs = smooth(t, v, smooth_s)              # de-noise (esp. the IMU)
            nv, vmin, vmax = _norm_mid(t, vs)
            shift = align.get(name, 0.0) if apply_align else 0.0
            ax.plot(t - t_ref - shift, nv, lw=1.2, color=colors[name],
                    label=f"{name}: {vmin:.4g}..{vmax:.4g} {unit}")
        ax.set_ylabel("normalized 0..1")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

    plot_panel(ax_raw, apply_align=False)
    ax_raw.set_title(f"{base} — BEFORE sync (common clock; legend shows true min..max "
                     f"over middle half)")
    plot_panel(ax_sync, apply_align=True)
    lag_txt = "  ".join(f"{n}{align[n]:+.3f}s" for n in ("camera", "imu"))
    ax_sync.set_title(f"AFTER sync (each shifted to the LiDAR: {lag_txt})")
    ax_sync.set_xlabel("time on common lidar clock, since t_ref (s)")
    fig.tight_layout()

    out = args.out or os.path.join(BAGS_DIR, base + "_sync.png")
    fig.savefig(out, dpi=120)
    print(f"\n-> {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
