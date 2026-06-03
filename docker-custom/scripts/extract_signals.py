#!/usr/bin/env python3
"""Reduce a timing-sync bag to one scalar per sensor sample.

Reads a rosbag2 recorded by record_sync_bag.sh and extracts:

  LiDAR  /rslidar_points        -> distance to the wall  (RANSAC plane, metres)
  Camera /image_raw/compressed  -> checkerboard center   (u or v, pixels)
  IMU    /rslidar_imu_data_fixed-> linear accel on motion axis (m/s^2)

Each series keeps its OWN header.stamp (the three sensors are on different
clocks -- that offset is exactly what we study). Output:

  bags/<name>_signals.npz   arrays: lidar_t, lidar_v, cam_t, cam_v, imu_t, imu_v
                            scalars: *_t0 (first abs stamp), meta (axis, etc.)
  bags/<name>_signals.csv   long format: sensor,t_abs,t_rel,value

Usage:
  python3 extract_signals.py <bag_name> [--config /opt/custom/config/sync.yaml]
"""
import argparse
import csv
import os
import sys

import numpy as np
import yaml

import cv2

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

BAGS_DIR = "/opt/custom/bags"
DEFAULT_CFG = "/opt/custom/config/sync.yaml"


def _stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


# ---------------------------------------------------------------- LiDAR -------
def lidar_features(msg, cfg):
    """Over a forward ROI, RANSAC-fit the wall plane. Returns
    (distance, cx, cy, cz): distance from sensor origin to the plane (m) and the
    centroid of the plane's inliers (m, sensor frame -- shows which axis the rig
    translated). All NaN if no good fit."""
    import open3d as o3d

    NAN4 = (float("nan"),) * 4
    offsets = {f.name: f.offset for f in msg.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        return NAN4
    dt = np.dtype({
        "names":   ["x", "y", "z"],
        "formats": [np.float32, np.float32, np.float32],
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": msg.point_step,
    })
    cloud = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    pts = np.column_stack([cloud["x"], cloud["y"], cloud["z"]]).astype(np.float64)
    pts = pts[np.isfinite(pts).all(axis=1)]

    lo = np.asarray(cfg["lidar"]["crop_xyz_min"])
    hi = np.asarray(cfg["lidar"]["crop_xyz_max"])
    pts = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
    if len(pts) < cfg["lidar"]["plane_min_inliers"]:
        return NAN4

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)
    model, inliers = pc.segment_plane(
        distance_threshold=cfg["lidar"]["plane_ransac_distance"],
        ransac_n=3, num_iterations=200)
    if len(inliers) < cfg["lidar"]["plane_min_inliers"]:
        return NAN4
    a, b, c, d = model                       # plane: ax+by+cz+d = 0
    dist = float(abs(d) / np.linalg.norm((a, b, c)))     # |distance origin->plane|
    cx, cy, cz = pts[inliers].mean(axis=0)
    return dist, float(cx), float(cy), float(cz)


# --------------------------------------------------------------- Camera -------
def _make_undistorter(caminfo):
    K = np.asarray(caminfo.k, dtype=np.float64).reshape(3, 3)
    D = np.asarray(caminfo.d, dtype=np.float64).reshape(-1, 1)
    return K, D


def _to_gray(msg):
    """Decode either sensor_msgs/CompressedImage or sensor_msgs/Image to gray."""
    if hasattr(msg, "format"):                      # CompressedImage (MJPEG/PNG)
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
    # raw sensor_msgs/Image
    enc = (msg.encoding or "").lower()
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    arr = arr.reshape(msg.height, msg.step)[:, :msg.width * (3 if "rgb" in enc or "bgr" in enc else 1)]
    if enc in ("rgb8", "bgr8"):
        arr = arr.reshape(msg.height, msg.width, 3)
        code = cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(arr, code)
    if enc in ("mono8", "8uc1", ""):
        return arr.reshape(msg.height, msg.width)
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def checkerboard_features(msg, cfg, undist):
    """Per frame, returns (u, v, scale, width):
      u, v   -- mean inner-corner pixel (board center)
      scale  -- RMS distance of corners from their centroid (px)
      width  -- apparent board width = horizontal span of inner corners (px),
                i.e. max(corner_x) - min(corner_x); grows as the board nears.
    All NaN if the board isn't found."""
    img = _to_gray(msg)
    if img is None:
        return (float("nan"),) * 4
    cols = cfg["checkerboard"]["cols"]
    rows = cfg["checkerboard"]["rows"]
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(img, (cols, rows), flags)
    if not found:
        return (float("nan"),) * 4
    pts = corners.reshape(-1, 2)
    if cfg["checkerboard"].get("undistort") and undist is not None:
        K, D = undist
        if cfg["_caminfo_model"] == "equidistant":
            pts = cv2.fisheye.undistortPoints(pts.reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)
        else:
            pts = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)
    c = pts.mean(axis=0)
    scale = float(np.sqrt(np.mean(np.sum((pts - c) ** 2, axis=1))))   # RMS radius, px
    width = float(pts[:, 0].max() - pts[:, 0].min())                  # horizontal span, px
    return float(c[0]), float(c[1]), scale, width


def pick_cam_signal(u, v, scale, width, cfg):
    sig = str(cfg["checkerboard"].get("signal", "scale")).lower()
    table = {"u": u, "v": v, "scale": scale, "width": width}
    return table[sig] if sig in table else _raise(f"unknown checkerboard.signal: {sig}")


def _raise(msg):
    raise ValueError(msg)



# ------------------------------------------------------------------ main ------
def _storage_id(bag_dir):
    """Read the storage plugin from metadata.yaml (Humble needs it explicitly;
    empty-string auto-detect is only reliable on Iron+). Default sqlite3."""
    meta = os.path.join(bag_dir, "metadata.yaml")
    if os.path.isfile(meta):
        with open(meta) as f:
            m = yaml.safe_load(f)
        try:
            return m["rosbag2_bagfile_information"]["storage_identifier"]
        except (KeyError, TypeError):
            pass
    return "sqlite3"


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=_storage_id(bag_dir)),
        rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_name", help="bag dir name under bags/ (or a full path)")
    ap.add_argument("--config", default=DEFAULT_CFG)
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    bag_dir = args.bag_name
    if not os.path.isabs(bag_dir):
        bag_dir = os.path.join(BAGS_DIR, bag_dir)
    if not os.path.isdir(bag_dir):
        sys.exit(f"bag dir not found: {bag_dir}")

    t = cfg["topics"]
    image_topic, cloud_topic = t["image"], t["cloud"]
    imu_topic, caminfo_topic = t["imu"], t["camera_info"]

    # Pass 1 for camera_info (model + intrinsics for optional undistort) and to
    # decide the IMU axis if 'auto'.
    reader, types = open_reader(bag_dir)
    caminfo = None
    imu_xyz = []
    cfg["_caminfo_model"] = "plumb_bob"
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == caminfo_topic and caminfo is None:
            caminfo = deserialize_message(data, get_message(types[topic]))
            cfg["_caminfo_model"] = caminfo.distortion_model or "plumb_bob"
        elif topic == imu_topic:
            m = deserialize_message(data, get_message(types[topic]))
            imu_xyz.append((m.linear_acceleration.x,
                            m.linear_acceleration.y,
                            m.linear_acceleration.z))

    axis_cfg = str(cfg["imu"]["axis"]).lower()
    if axis_cfg == "auto":
        if not imu_xyz:
            sys.exit(f"no IMU messages on {imu_topic}")
        axis_idx = int(np.argmax(np.var(np.asarray(imu_xyz), axis=0)))
    else:
        axis_idx = {"x": 0, "y": 1, "z": 2}[axis_cfg]
    axis_name = "xyz"[axis_idx]
    undist = _make_undistorter(caminfo) if caminfo is not None else None

    # Pass 2: extract ALL candidate channels per sensor (so inspect_axes.py can
    # show which axis dominates), keeping each frame's own stamp.
    reader, types = open_reader(bag_dir)
    lidar_t, lx, ly, lz, lidar_v = ([] for _ in range(5))
    cam_t, cu, cv, cs, cw = ([] for _ in range(5))
    imu_t, ax, ay, az = ([] for _ in range(4))
    n_cam_seen = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == cloud_topic:
            dist, x, y, z = lidar_features(msg, cfg)
            if np.isfinite(dist):
                lidar_t.append(_stamp_s(msg.header))
                lidar_v.append(dist); lx.append(x); ly.append(y); lz.append(z)
        elif topic == image_topic:
            n_cam_seen += 1
            u, v, scale, width = checkerboard_features(msg, cfg, undist)
            if np.isfinite(scale):
                cam_t.append(_stamp_s(msg.header))
                cu.append(u); cv.append(v); cs.append(scale); cw.append(width)
        elif topic == imu_topic:
            imu_t.append(_stamp_s(msg.header))
            ax.append(msg.linear_acceleration.x)
            ay.append(msg.linear_acceleration.y)
            az.append(msg.linear_acceleration.z)

    A = np.asarray
    lidar_t, lx, ly, lz, lidar_v = map(A, (lidar_t, lx, ly, lz, lidar_v))
    cam_t, cu, cv, cs, cw = map(A, (cam_t, cu, cv, cs, cw))
    imu_t, ax, ay, az = map(A, (imu_t, ax, ay, az))

    # Reduced chosen signals (what plot_sync.py aligns).
    cam_v = pick_cam_signal(cu, cv, cs, cw, cfg) if len(cam_t) else cu
    imu_v = (ax, ay, az)[axis_idx]

    def t0(a): return float(a[0]) if len(a) else 0.0

    # --- COMMON CLOCK ---
    # LiDAR + IMU are already on the AIRY hardware clock (use_lidar_clock:true).
    # The camera is on the host Unix clock, ~1.78e9 s ahead. We take that gross
    # EPOCH out of the camera stamps -- estimated as the recording-start gap
    # (cam_t0_raw - lidar_t0_raw); the sensor preflight guarantees both were live
    # when recording began, so their first stamps are the same real instant to
    # within ~a frame. After this shift ALL stamps are on the common (lidar)
    # clock and the only camera<->lidar difference left is the real sub-second
    # lag the reversal sync measures. cam_epoch + cam_t0_raw are saved for record.
    cam_t0_raw = t0(cam_t)
    cam_epoch = (cam_t0_raw - t0(lidar_t)) if (len(cam_t) and len(lidar_t)) else 0.0
    cam_t = cam_t - cam_epoch            # camera -> common (lidar) clock

    out = os.path.join(BAGS_DIR, os.path.basename(bag_dir.rstrip("/")) + "_signals")
    np.savez(out + ".npz",
             # reduced signals used by plot_sync.py (ALL on the common lidar clock)
             lidar_t=lidar_t, lidar_v=lidar_v, lidar_t0=t0(lidar_t),
             cam_t=cam_t, cam_v=cam_v, cam_t0=t0(cam_t),
             cam_axis=str(cfg["checkerboard"].get("signal", "scale")),
             imu_t=imu_t, imu_v=imu_v, imu_t0=t0(imu_t), imu_axis=axis_name,
             # epoch removed from the camera to reach the common clock
             cam_epoch=cam_epoch, cam_t0_raw=cam_t0_raw,
             # all candidate channels for inspect_axes.py
             lidar_cx=lx, lidar_cy=ly, lidar_cz=lz, lidar_dist=lidar_v,
             cam_u=cu, cam_v_px=cv, cam_scale=cs, cam_width=cw,
             imu_ax=ax, imu_ay=ay, imu_az=az)

    with open(out + ".csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["sensor", "t_abs", "t_rel", "value"])
        for name, ts, vs in (("lidar", lidar_t, lidar_v),
                             ("camera", cam_t, cam_v),
                             ("imu", imu_t, imu_v)):
            b = t0(ts)
            for ti, vi in zip(ts, vs):
                w.writerow([name, f"{ti:.9f}", f"{ti - b:.9f}", f"{vi:.6f}"])

    print(f"lidar : {len(lidar_t):5d} pts  (distance, m)")
    print(f"camera: {len(cam_t):5d} pts  (checkerboard {cfg['checkerboard'].get('signal','scale')}, px)"
          f"  [board found in {len(cam_t)}/{n_cam_seen} frames]")
    print(f"imu   : {len(imu_t):5d} pts  (accel axis '{axis_name}', m/s^2)")
    print(f"camera epoch removed: {cam_epoch:.6f} s  "
          f"(camera now on the common lidar clock)")
    print(f"-> {out}.npz")
    print(f"-> {out}.csv")
    if len(cam_t) == 0:
        print("WARNING: checkerboard never detected -- check cols/rows in sync.yaml "
              "(inner corners, not squares) and that the board is in frame.",
              file=sys.stderr)


if __name__ == "__main__":
    main()
