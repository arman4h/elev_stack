# CLAUDE.md — Handoff: Livox (45° tilt) + ZED 2i → CUDA elevation map → rviz2

You are taking over a partly finished ROS2 project. Goal of your work: make it **ready for the first
test with the real lidar** on the owner's Jetson. Read this whole file before changing anything.

## 1. Goal (owner's words, condensed)
- Livox 360-class lidar (assume **Mid-360**) mounted **tilted 45° forward**. Need point cloud data in
  `base_link` ("zero-degree" frame) to build an elevation map.
- Tilt handled through ROS2 configuration (static TF) rather than ad-hoc math in the mapper.
- Elevation mapper must be **entirely Python** and **CUDA-accelerated**. Everything runs on a **Jetson**.
- Final visual check in **rviz2** with a proper elevation map.
- Odometry source: **ZED 2i** (visual-inertial odometry).

## 2. Environment (fixed constraints)
- Jetson **Orin**, **JetPack 5.x** (Ubuntu 20.04, L4T 35.x, CUDA 11.4, system Python 3.8).
- **ROS2 Humble**. Humble debs do not exist for 20.04 → run in a **Docker container** (base built from
  source, e.g. `dustynv/ros:humble-ros-base-l4t-r35.x`). Exact JetPack minor version is unknown:
  ask the owner or detect with `cat /etc/nv_tegra_release`.
- Owner is in Dhaka, a CS student on a Mars-rover team; has hardware access, **you probably do not**.
  Never claim something was run on hardware unless you actually ran it.
- Keep all code **Python 3.8 compatible** (no `X | Y` type unions, no `match`).
- Mapper stays pure Python + CuPy (NumPy fallback). No C++ nodes for the mapper.

## 3. Architecture
```
Livox Mid-360 --UDP--> livox_ros_driver2 --> /livox/lidar (PointCloud2, frame livox_frame)
ZED 2i --> zed-ros2-wrapper --> TF odom -> <zed frame>
static TFs:  <zed frame> -> base_link (inverse of measured base_to_zed)
             base_link  -> livox_frame (pitch +45°, from config/mount.yaml)
elevation_mapper (rclpy + CuPy):
   cloud -> GPU -> filter(range, finite) -> livox_frame->base_link -> self-filter box
         -> base_link->odom -> height window (relative to robot z) -> recenter rolling grid
         -> per-cell max/mean (scatter ops) -> Kalman fusion + reset gate + forget -> grid
   timer @5 Hz -> /elevation_map/cloud (x,y,z, intensity=slope deg, variance) in frame odom
rviz2 (preferably on a laptop, same ROS_DOMAIN_ID, Cyclone DDS)
```
Design decisions (keep unless you have a strong reason):
- Fixed frame is **odom**, never `map` (map can jump on relocalization and tear a rolling map).
- TF is the single source of truth for the mount; exactly **one parent per frame**.
- Core logic in `elevation_core.py` has **no ROS imports** so it is testable offline on CPU.
- `use_msg_stamp=false`: uses latest TF, because Livox stamps may not be wall-clock. Revisit after
  checking real stamps.
- Pitch sign: REP-103, positive pitch about +y tilts the sensor z-axis toward +x. Owner's mount =
  +0.7853981634 rad nominal; must be confirmed with `check_floor`.

## 4. Repository map (workspace root = this folder)
- `src/elev_bringup/` ament_python package
  - `elev_bringup/elevation_core.py` MapperConfig, ElevationGrid (recenter/integrate/to_points), process_cloud, backend select
  - `elev_bringup/mapper_node.py` ROS node; `cloud_xyz()` reads x,y,z by field offsets (works for any point_step)
  - `elev_bringup/check_floor.py` mount calibration helper (plane fit on lowest 30% of points)
  - `elev_bringup/fake_lidar.py`, `sim_world.py` synthetic robot/world (floor, 0.5 m box, 16.7° ramp)
  - `elev_bringup/tf_math.py` rpy<->R, transform inverse
  - `launch/sensors.launch.py` Livox + ZED + static TFs; `mapping.launch.py` mapper (+rviz);
    `full.launch.py` all; `sim.launch.py` hardware-free
  - `config/mount.yaml` (sensor mounts), `mapper.yaml`, `MID360_config.json` (lidar IP `192.168.1.1XX` placeholder!),
    `zed_override.yaml`, `elevation.rviz`
- `docker/Dockerfile`, `docker/entrypoint.sh`, `compose.yaml` container setup
- `tools/offline_test.py` ROS-free end-to-end test (`PYTHONPATH=src/elev_bringup python3 tools/offline_test.py cpu|cuda`)
- `VERIFY.md` 7-step hardware checklist; `README.md` quick start

## 5. What is verified vs not
**Verified (on a CPU-only sandbox, no ROS installed):**
- All Python compiles; `offline_test.py cpu` passes: floor z≈0.003 m, box top 0.516 m (true 0.5),
  ramp slope 16.7° (true 16.7°), ~2.7 ms/frame for ~18k points on CPU; rolling window follows robot.
- tf_math round-trips (rpy→R→rpy, inverse).

**NOT verified (treat as likely to need fixes, do not assume correct):**
1. Docker build on the Orin: base image tag existence, missing ROS packages on 20.04
   (e.g. `pcl_conversions`/perception_pcl, `diagnostic_updater`, `xacro`, `robot_state_publisher` and
   whatever the ZED wrapper needs — use rosdep/vcs, or an Isaac ROS / jetson-containers base if better),
   Python 3.8 + `cupy-cuda11x` wheel compatibility, ZED SDK installer URL pattern/version vs L4T minor,
   `colcon build` of livox_ros_driver2 (needs `cp package_ROS2.xml package.xml`, `-DROS_EDITION=ROS2 -DHUMBLE_ROS=humble`).
2. Whole package has **never been built with colcon** or launched under ROS2.
3. CuPy path: `cupyx.scatter_add` / `cupyx.scatter_max` on float32, `xp.roll`, `xp.gradient` are
   untested on GPU. `tools/offline_test.py cuda` must pass on the Orin.
4. ZED wrapper: which frame it publishes (`odom -> zed_camera_link` vs `odom -> base_link`), launch
   argument names (`camera_model`, `camera_name`, `publish_map_tf`, `ros_params_override_path`) and the
   keys in `zed_override.yaml` were written from memory for 4.x — **check the installed wrapper's
   launch file and common yaml**. If it owns `odom -> base_link`, delete the `<zed frame> -> base_link` static TF.
5. livox_ros_driver2: executable name (`livox_ros_driver2_node`), parameter names, `xfer_format: 0`
   field layout, `MID360_config.json` structure — check against the checked-out driver version.
6. `elevation.rviz` was hand-written; confirm it loads in rviz2 Humble and fix key names.
7. Livox timestamps vs ROS time (VERIFY.md step 3) and PTP/host-time options unknown.
8. `check_floor.py` relies on `sensor_msgs_py.point_cloud2.read_points` returning a structured array (Humble behaviour).
9. Mount values: only pitch is set; offsets, roll/yaw, and `base_to_zed` are zeros (owner must measure).

## 6. Your task list (priority order)
1. **Make it build.** Fix Dockerfile/compose until `docker compose build` succeeds on Orin JetPack 5.x, ZED SDK
   and Livox SDK2 installed, workspace builds with `colcon build --symlink-install`. Document exact tags/URLs.
2. **Static checks you can do without hardware:** `colcon build`, `colcon test`/flake8 if available,
   run `sim.launch.py backend:=cpu` (use rviz or `ros2 topic echo`), confirm `/elevation_map/cloud`
   shows box + ramp. Add a `launch_testing` or script-based smoke test if feasible.
3. **Reconcile upstream APIs** (items 4–6 above) by reading the real sources/launch files in the container;
   correct `sensors.launch.py`, `zed_override.yaml`, `MID360_config.json`, `elevation.rviz`.
4. **GPU path:** run `offline_test.py cuda` on Orin; fix any CuPy issue; then profile (`nsys`/timers). Target
   mean < 20 ms per cloud at 10 Hz alongside the ZED SDK. Candidate optimizations only if needed: pinned
   host buffers, preallocated per-frame scratch arrays, avoiding boolean-mask syncs, a RawKernel for binning.
5. **Operational helpers:** script to set the Jetson Ethernet to 192.168.1.5/24 and verify lidar reachability;
   a `diagnose.sh` printing topic rates, TF tree (`view_frames`), stamp sanity, GPU/CPU load (`tegrastats`);
   `nvpmodel`/`jetson_clocks` notes; Cyclone DDS config for laptop rviz.
6. **Calibration workflow:** make `check_floor` print a suggested pitch/roll correction and the floor height;
   document the turn-in-place smear test. Add a small tool to write measured values into `mount.yaml`.
7. Update `VERIFY.md`/`README.md` to match what you actually verified. Keep an "unverified" list honest.

Optional later (do not start unless asked): FAST-LIO2 odometry swap, grid_map_msgs output, traversability
layer from slope/variance, IMU deskew, Nav2 costmap bridge, per-cell multi-hit height model.

## 7. Definition of "ready for real-lidar testing"
- [ ] Container builds on Orin; `ros2 pkg list | grep -E "elev_bringup|livox_ros_driver2|zed_wrapper"` shows all three
- [ ] `ros2 launch elev_bringup sim.launch.py backend:=cpu` runs; map shows box ~0.5 m and ramp ~17°
- [ ] `tools/offline_test.py cuda` passes on Orin; mapper mean processing < 20 ms/frame
- [ ] `sensors.launch.py` yields one-parent TF tree odom→<zed>→base_link→livox_frame; frame names verified
- [ ] Network/IP setup for Mid-360 scripted and documented; lidar topic at ~10 Hz
- [ ] Timestamp policy decided and documented (`use_msg_stamp`)
- [ ] 10-minute soak run of `full.launch.py` without crash/leak (when hardware available; otherwise sim soak)
- [ ] VERIFY.md steps match the real launch/topic names; unverified items clearly listed

## 8. Rules for you
- Do not invent parameter/topic names for upstream packages — read their source/docs in the container.
- Do not silently change the TF design, the fixed frame, or the Python+CuPy constraint; if you must, say why.
- Keep `elevation_core.py` ROS-free and CPU-testable. Re-run `offline_test.py cpu` after every core change.
- Tell the owner plainly what you could not test without the real lidar/ZED/GPU.
- Open questions to ask the owner if blocked: exact JetPack/L4T version; lidar serial (for IP 192.168.1.1XX);
  measured mount offsets of Livox and ZED relative to base_link; robot footprint for `self_filter`;
  whether rviz2 runs on the Jetson or a laptop.