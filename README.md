# elev_bringup: elevation map node (ROS 2 Humble)

A single ROS 2 node that turns the Livox lidar point cloud into a rolling 2.5D elevation map
and publishes it. Lidar driver, ZED, and the static TFs are set up separately on the robot.

## Inputs and outputs

| Direction | Topic / TF | Type | Notes |
|---|---|---|---|
| in | `/livox/lidar` | `sensor_msgs/PointCloud2` | frame `livox_frame`, needs x,y,z float32 |
| in | TF `base_link` ← `livox_frame` | static | mount (45° pitch) |
| in | TF `odom` ← `base_link` | dynamic | from the ZED odometry |
| out | `/elevation_map/cloud` | `sensor_msgs/PointCloud2` | frame `odom`, 5 Hz |

Output fields per cell: `x`, `y`, `z` (height), `intensity` (slope in degrees), `variance`.

The map is fixed in `odom` and rolls with the robot. The fixed frame is `odom`, not `map`.

## Build (on the Jetson, in your existing ROS 2 workspace)

```bash
cd ~/your_ws/src
ln -s ~/elev_stack/src/elev_bringup .   # or copy the folder
cd ~/your_ws
colcon build --symlink-install --packages-select elev_bringup
source install/setup.bash
```

Use `--symlink-install` so Python edits take effect without rebuilding.

## Run

```bash
ros2 launch elev_bringup elevation.launch.py                   # node only
ros2 launch elev_bringup elevation.launch.py rviz:=true        # plus rviz2 view
ros2 launch elev_bringup elevation.launch.py backend:=cpu      # force NumPy (no GPU)
```

Check the output:
```bash
ros2 topic hz /elevation_map/cloud        # ~5 Hz
ros2 topic echo /elevation_map/cloud --once
```

## Configuration

All settings are in `config/mapper.yaml`:

- `cloud_topic`, `fixed_frame`, `base_frame`: input topic and frames.
- `resolution`, `size_m`: cell size and map width (metres).
- `measure`: `max` (obstacle-friendly) or `mean` (smoother).
- `min_range`, `max_range`: point range filter (metres).
- `z_min_rel`, `z_max_rel`: height window relative to the robot.
- `self_filter`: base_link box to ignore (robot body), `[xmin,xmax,ymin,ymax,zmin,zmax]`, zeros = off.
- `var_base`, `range_var_coeff`, `process_var`, `forget_var`, `gate_sigma`: noise, drift, and reset tuning.
- `publish_hz`: output rate.
- `use_msg_stamp`: `false` uses the latest TF. Set `true` only once lidar timestamps are confirmed to match ROS time.
- `backend`: `auto`, `cuda`, or `cpu` (passed by the launch file).

## Tests (no ROS or GPU needed)

```bash
cd src/elev_bringup
python3 -m unittest discover -s test -v
```

## Verified so far

- Builds with colcon (ROS 2 Jazzy in a test sandbox, CPU only).
- The node and launch file run end to end against a simulated lidar and TF: output at 5 Hz, 4–7 ms per frame on CPU.
- Unit tests pass.

## Not yet verified on the robot

- CuPy/GPU path (`backend:=cuda`).
- Real Livox cloud and real TF timing.
- Lidar timestamps vs ROS time (`use_msg_stamp`).
- Tuning values for your environment.
