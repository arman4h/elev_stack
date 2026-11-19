# elev_bringup

Elevation map node. Publishes `/elevation_map/cloud` from `/livox/lidar`.

## Setup

1. Make sure the lidar driver, ZED, and static TFs are already running. You need:
   - `/livox/lidar` (PointCloud2, frame `livox_frame`)
   - TF `odom` → `base_link` and `base_link` → `livox_frame`

2. Put the package in your ROS 2 workspace:
   ```bash
   cd ~/your_ws/src
   ln -s ~/elev_stack/src/elev_bringup .
   ```

3. Build and source:
   ```bash
   cd ~/your_ws
   colcon build --symlink-install --packages-select elev_bringup
   source install/setup.bash
   ```

## Publishing

1. Start the node:
   ```bash
   ros2 launch elev_bringup elevation.launch.py
   ```
   Options: `rviz:=true` opens RViz, `backend:=cpu` forces NumPy (no GPU).

2. Check the output:
   ```bash
   ros2 topic hz /elevation_map/cloud
   ros2 topic echo /elevation_map/cloud --once
   ```

| Topic | Type | Frame | Rate |
|---|---|---|---|
| `/elevation_map/cloud` (out) | `sensor_msgs/PointCloud2` | `odom` | 5 Hz |

Fields: `x`, `y`, `z` (height), `intensity` (slope, degrees), `variance`.

Settings are in `src/elev_bringup/config/mapper.yaml`.
