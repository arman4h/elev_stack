# Verification checklist (first real-lidar test)

Legend: [sandbox] = done and passing in the CPU sandbox (ROS 2 Jazzy, no GPU/hardware).
[hardware] = still needs the Orin, Mid-360 and ZED 2i.

## Done in the sandbox
- [sandbox] `colcon build --symlink-install` for elev_bringup succeeds.
- [sandbox] `sim.launch.py backend:=cpu` runs: map publishes at 5 Hz, ~7 ms per frame, 85 s with no errors.
- [sandbox] `tools/test_core.py` (7 tests) and `offline_test.py cpu` pass.
- [sandbox] Calibration correction converges to the true mount (checked numerically).
- [sandbox] `rviz2 -d config/elevation.rviz` loads with no config parse errors (GL window not checked).
- [sandbox] `sensors.launch.py` refuses to start while the lidar IP placeholder is still present.

## Checklist on the robot
0. [hardware] Network: `scripts/jetson_network.sh <eth-iface> <lidar-serial>` sets 192.168.1.5/24 and
   pings the lidar. Then `python3 tools/set_lidar_ip.py <lidar-serial>` (IP = 192.168.1.1 + last 2 digits).
1. [hardware] Build the container: `docker compose build`. Expect to iterate. Base tag, ZED SDK URL,
   and rosdep keys are unverified. The ZED wrapper is pinned to `humble-v4.2.x`.
2. [hardware] `ros2 topic hz /livox/lidar` gives ~10 Hz. `ros2 topic hz /livox/imu` gives ~200 Hz.
3. [hardware] Timestamps: `scripts/diagnose.sh` compares the lidar stamp with the wall clock.
   `use_msg_stamp` stays false until the stamps are confirmed sane.
4. [hardware] TF: `ros2 run tf2_tools view_frames`. Required chain: odom -> zed_camera_link -> base_link ->
   livox_frame, each frame with ONE parent. `publish_urdf` is off in sensors.launch.py so the ZED
   description cannot add a second parent to base_link.
5. [hardware] `ros2 run tf2_ros tf2_echo odom base_link` moves smoothly as the robot moves.
6. [hardware] Floor calibration: robot on flat ground, `ros2 run elev_bringup check_floor`, then
   `python3 tools/set_mount.py base_to_livox ...` with the suggested values. Relaunch. Repeat until tilt ~0 deg.
   The floor z at base origin should be ~0 when base_link sits on the ground.
7. [hardware] Turn-in-place test in rviz2 (fixed frame odom, show /livox/lidar): a wall must stay a sharp
   line. Smearing points to yaw/offset error in base_to_zed or base_to_livox.
8. [hardware] `offline_test.py cuda` passes on the Orin. Mean processing time < 20 ms per cloud.
9. [hardware] 10-minute soak: `ros2 launch elev_bringup full.launch.py`, watch the mapper report and memory.

## Unverified (be explicit when reporting results)
- Docker build on the Orin (base tag `dustynv/ros:humble-ros-base-l4t-r35.4.1`, ZED SDK 4.2 URL, rosdep keys).
- CuPy path on GPU (`cupyx.scatter_max`, `xp.roll`, `xp.gradient` on float32). Only the NumPy path ran.
- Livox driver against the real lidar (point layout, timestamps, MID360 JSON fields beyond the ones we set).
- ZED wrapper against the real camera: frame names (`zed_camera_link`), odom publishing, IMU fusion.
  The wrapper's `zed_descr.urdf.xacro` could not be fetched, so `publish_urdf:=false` is a design choice
  from the TF rules, not a checked fact about the wrapper.
- rviz2 rendering (no display in the sandbox).
- Lidar timestamps vs ROS time, and whether PTP is needed.
- Mount values (only pitch 45 deg is nominal; offsets and `base_to_zed` are zeros until measured).
