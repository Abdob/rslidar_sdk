# docker — RoboSense AIRY stdout demo

Minimal Docker setup that runs `rs_driver`'s standalone `demo_online` against a
live RoboSense AIRY LiDAR. No ROS, no DeepStream — just prints the per-frame
point count to stdout. Use this to confirm the LiDAR is reachable and emitting
before bringing up anything heavier.

## What it builds

- Base: `ubuntu:22.04`
- Compiles `src/rs_driver/` with `COMPILE_DEMOS=ON`
- Runs `demo_online`, which is hardcoded for `RSAIRY` on ports `6699` (MSOP),
  `7788` (DIFOP), `6688` (IMU)

## Prereqs

- Docker
- AIRY reachable on the host network
- The AIRY's **destination IP** must be set (in the LiDAR's web UI) to an IP
  that exists on your host's NIC. Confirm with:
  ```
  ip -4 addr show
  ```
  If the LiDAR is shooting at e.g. `192.168.1.123` but your NIC is `.135`,
  either reconfigure the LiDAR or add the dest IP as an alias:
  ```
  sudo ip addr add 192.168.1.123/24 dev <iface>
  ```

## Build

```
./docker/docker_build.sh
```
Tags the image `rslidar-airy-demo`.

## Run

```
./docker/docker_run.sh
```
`--net=host` is required — the container needs to receive UDP on the host's
network stack. Output looks like:
```
msg: 0  point cloud size: 86400
msg: 1  point cloud size: 86400
...
```
At ~10 Hz with ~86k points per frame, you're seeing the full AIRY stream.

Ctrl-C stops it.

## Customize

The lidar type and ports are hardcoded in
[../src/rs_driver/demo/demo_online.cpp:194-200](../src/rs_driver/demo/demo_online.cpp#L194-L200).
Change there and re-run `docker_build.sh`.

## Notes

- The Dockerfile passes `-include memory -include functional` to the rs_driver
  build. The vendored `input.hpp` uses `std::shared_ptr` without including
  `<memory>`, which fails under gcc 11+. Don't drop those flags.
- If you see only `ERRCODE_MSOPTIMEOUT` lines and no point clouds, the LiDAR
  is reachable (ping OK) but its destination IP isn't on this host — see the
  Prereqs note above.
