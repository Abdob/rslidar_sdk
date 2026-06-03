#!/usr/bin/env python3
"""Estimate the LiDAR<->IMU time offset by CROSS-CORRELATION (not reversal
pairing). Uses the whole continuous signal, so it's far more consistent.

LiDAR measures position (distance to wall); the IMU measures acceleration. To
correlate them we bring both to the same derivative order and try all three as a
consistency check -- a trustworthy result is when the three agree:

  velocity : v_lidar = d/dt(distance)         vs  v_imu = integral(accel)
  accel    : a_lidar = d^2/dt^2(distance)      vs  a_imu = accel
  position : x_lidar = distance                vs  x_imu = integral^2(accel)

Each pair is detrended, resampled to a common uniform grid (on the shared lidar
clock), z-normalized, and cross-correlated over +/- max_lag. The lag at the
correlation peak (parabolically refined) is the offset:

    offset = t_imu - t_lidar  for the same physical motion
    (positive => the IMU stamps the motion LATER than the LiDAR)

Usage:
  python3 lidar_imu_offset.py <bag_name> [--axis x|y|z|auto] [--max-lag 1.0]
                              [--grid-hz 200] [--method velocity] [--show]
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("TkAgg" if "--show" in sys.argv else "Agg")
import matplotlib.pyplot as plt
from scipy.signal import detrend, savgol_filter, correlate, correlation_lags
from scipy.integrate import cumulative_trapezoid

BAGS_DIR = "/opt/custom/bags"
METHODS = ("velocity", "accel", "position")


def _resample(t, v, grid):
    return np.interp(grid, t, v)


def _odd(n):
    n = int(n)
    return n if n % 2 == 1 else n + 1


def lidar_signal(t, dist, method, grid):
    """Bring lidar distance to the requested derivative order, on `grid`."""
    d = _resample(t, dist, grid)
    dt = grid[1] - grid[0]
    if method == "position":
        return detrend(d)
    # Savitzky-Golay gives a smooth derivative of the low-rate, noisy distance.
    win = _odd(max(5, min(len(d) // 2, int(0.4 / dt))))   # ~0.4 s window
    if method == "velocity":
        return savgol_filter(d, win, 2, deriv=1, delta=dt)
    return savgol_filter(d, win, 2, deriv=2, delta=dt)      # accel


def imu_signal(t, acc, method, grid):
    """Bring imu acceleration to the requested derivative order, on `grid`.
    Integration constants/biases are removed by detrending (linear)."""
    a = _resample(t, acc - np.mean(acc), grid)
    if method == "accel":
        return a
    v = cumulative_trapezoid(a, grid, initial=0.0)
    v = detrend(v)                          # kill accel-bias velocity ramp
    if method == "velocity":
        return v
    x = cumulative_trapezoid(v, grid, initial=0.0)
    return detrend(x)                        # position (double integral)


def _z(v):
    s = v.std()
    return (v - v.mean()) / s if s > 1e-12 else v - v.mean()


def cross_corr_offset(sig_l, sig_i, dt, max_lag):
    """Lag (s) that best aligns the IMU signal to the LiDAR signal, by
    cross-correlation. offset = t_imu - t_lidar. Also returns the (signed) peak
    correlation and the (lags_s, corr) curve for plotting."""
    a, b = _z(sig_l), _z(sig_i)
    corr = correlate(a, b, mode="full")
    lags = correlation_lags(len(a), len(b), mode="full")
    corr /= len(a)                                   # ~normalized
    keep = np.abs(lags * dt) <= max_lag
    lags, corr = lags[keep], corr[keep]
    # Peak by |corr| (the IMU axis sign may be flipped vs lidar forward).
    p = int(np.argmax(np.abs(corr)))
    lag = float(lags[p])
    # parabolic sub-sample refinement on |corr|
    if 0 < p < len(corr) - 1:
        c0, c1, c2 = abs(corr[p - 1]), abs(corr[p]), abs(corr[p + 1])
        denom = (c0 - 2 * c1 + c2)
        if abs(denom) > 1e-12:
            lag += 0.5 * (c0 - c2) / denom
    # offset = t_imu - t_lidar. correlate(a=lidar, b=imu) index lag L aligns
    # imu(n-L) with lidar(n); empirically (verified against an injected delay)
    # the IMU-later offset is -L*dt. Return the corr curve in that same
    # convention (x = t_imu - t_lidar), sorted for a clean plot.
    offset = -lag * dt
    x = -lags * dt
    order = np.argsort(x)
    return offset, float(corr[p]), x[order], corr[order]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_name")
    ap.add_argument("--axis", default=None, help="imu axis x|y|z|auto (default: as extracted)")
    ap.add_argument("--max-lag", type=float, default=1.0, help="search window +/- s")
    ap.add_argument("--grid-hz", type=float, default=200.0, help="resample rate")
    ap.add_argument("--method", default="velocity", choices=METHODS,
                    help="which order to plot (all three are always printed)")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    base = os.path.basename(args.bag_name.rstrip("/"))
    npz = args.bag_name if args.bag_name.endswith(".npz") else \
        os.path.join(BAGS_DIR, base + "_signals.npz")
    if not os.path.isfile(npz):
        sys.exit(f"signals not found: {npz}  (run extract_signals.py first)")
    d = np.load(npz, allow_pickle=True)

    lt, dist = np.asarray(d["lidar_t"]), np.asarray(d["lidar_dist"])
    it = np.asarray(d["imu_t"])
    if len(lt) < 5 or len(it) < 5:
        sys.exit("need both lidar and imu data")

    # Pick IMU axis.
    axis = (args.axis or str(d["imu_axis"])).lower()
    if axis == "auto":
        chans = {"x": d["imu_ax"], "y": d["imu_ay"], "z": d["imu_az"]}
        axis = max(chans, key=lambda k: np.var(chans[k]))
    acc = np.asarray(d[{"x": "imu_ax", "y": "imu_ay", "z": "imu_az"}[axis]])

    # Common uniform grid over the overlapping time span (shared lidar clock).
    g0, g1 = max(lt[0], it[0]), min(lt[-1], it[-1])
    dt = 1.0 / args.grid_hz
    grid = np.arange(g0, g1, dt)
    if len(grid) < 16:
        sys.exit("overlap window too short")

    print(f"\nLiDAR<->IMU offset by cross-correlation  (bag {base}, imu axis '{axis}')")
    print(f"  overlap {g1 - g0:.1f} s, grid {args.grid_hz:g} Hz, search +/-{args.max_lag:g} s")
    print(f"  convention: offset = t_imu - t_lidar  (positive => IMU stamps later)\n")
    print(f"  {'method':10s} {'offset [s]':>12s} {'peak corr':>11s}")
    results = {}
    for m in METHODS:
        sl = lidar_signal(lt, dist, m, grid)
        si = imu_signal(it, acc, m, grid)
        off, peak, lags_s, corr = cross_corr_offset(sl, si, dt, args.max_lag)
        results[m] = (off, peak, lags_s, corr, sl, si)
        print(f"  {m:10s} {off:12.4f} {peak:11.3f}")

    offs = np.array([results[m][0] for m in METHODS])
    print(f"\n  median offset: {np.median(offs):+.4f} s   spread (max-min): "
          f"{offs.max() - offs.min():.4f} s")
    print("  -> trust it when the three methods agree (small spread) and |peak corr| is high.")

    # Plot the chosen method: aligned signals + the correlation curve.
    off, peak, lags_s, corr, sl, si = results[args.method]
    fig, (a0, a1) = plt.subplots(2, 1, figsize=(12, 8))
    a0.plot(grid - grid[0], _z(sl), label=f"lidar {args.method}", color="tab:blue")
    a0.plot(grid - grid[0], _z(si), label=f"imu {args.method} (raw)",
            color="tab:red", alpha=0.45)
    a0.plot(grid - grid[0] - off, _z(si), label=f"imu shifted by {-off:+.3f}s",
            color="tab:green")
    a0.set_title(f"{base}: LiDAR vs IMU ({args.method}); offset t_imu - t_lidar = "
                 f"{off:+.4f} s")
    a0.set_xlabel("time on common lidar clock (s)"); a0.set_ylabel("z-norm")
    a0.legend(loc="upper right", fontsize=8); a0.grid(True, alpha=0.3)

    a1.plot(lags_s, corr, color="tab:purple")
    a1.axvline(off, color="k", ls="--", lw=1, label=f"peak @ {off:+.4f} s")
    a1.set_title("cross-correlation vs lag")
    a1.set_xlabel("lag = t_imu - t_lidar (s)"); a1.set_ylabel("correlation")
    a1.legend(loc="upper right", fontsize=8); a1.grid(True, alpha=0.3)
    fig.tight_layout()

    out = args.out or os.path.join(BAGS_DIR, base + "_lidar_imu.png")
    fig.savefig(out, dpi=120)
    print(f"\n-> {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
