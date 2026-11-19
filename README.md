# Livox Mid-360 (45 deg) + ZED 2i -> elevation map -> rviz2   (Jetson Orin, JetPack 5.x, ROS 2 Humble)

## Data flow
Livox `/livox/lidar` (frame `livox_frame`) --static TF base_link<-livox_frame (45 deg pitch)-->
GPU transform --TF odom<-base_link (ZED visual-inertial odometry)--> rolling 2.5D grid (CuPy,
NumPy fallback) --> `/elevation_map/cloud` (frame `odom`) --> rviz2.
Cloud fields: `x y z` (elevation), `intensity` (slope, deg), `variance`.

## Status
- Verified here (ROS 2 Jazzy sandbox, CPU only): colcon build, simulation end to end, 85 s soak,
  unit tests, offline end-to-end test, calibration math, rviz config loads without parse errors.
- Not verified: Docker build on the Orin, the real lidar, the real ZED, CuPy/GPU path.
  See `VERIFY.md` for the full list. Treat the hardware path as untested until VERIFY.md is run.

## Setup

### 1. Put the project on GitHub (laptop, one time)
```bash
cd elev_stack
git init -b main
git add .
git commit -m "elevation mapper: Livox 45deg + ZED 2i, sim + Docker setup"
git remote add origin git@github.com:<your-user>/elev_stack.git   # create the empty repo on GitHub first
git push -u origin main
```

### 2. Prepare the Jetson (one time)
**Check the JetPack version.** The Dockerfile expects L4T R35.4.1 (JetPack 5.1.x):
```bash
cat /etc/nv_tegra_release
```
If your release differs, change `BASE` in `docker/Dockerfile` and `ZED_SDK_URL` (the L4T minor must match).

**Install Docker and the compose plugin:**
```bash
sudo apt update
sudo apt install -y docker.io docker-compose-plugin git
sudo usermod -aG docker $USER
sudo systemctl enable --now docker
newgrp docker                    # or log out and back in
docker info | grep -i runtime    # must list "nvidia"
```
If `nvidia` is not listed: `sudo apt install nvidia-container`, then restart Docker.

**GitHub access (private repo).** Create an SSH key on the Jetson and add the public key to GitHub
(Settings → SSH keys):
```bash
ssh-keygen -t ed25519 -C "jetson"
cat ~/.ssh/id_ed25519.pub
ssh -T git@github.com            # should say "Hi <user>!"
```

### 3. Clone on the Jetson
```bash
cd ~
git clone git@github.com:<your-user>/elev_stack.git
cd elev_stack
```

### 4. Connect the lidar
Find the Ethernet interface, then configure it and check the lidar answers. Power the Mid-360
(9–27 V) and wire it directly to this port:
```bash
ip -br link                                      # e.g. eth0 or enP8p1s0
scripts/jetson_network.sh <iface> <lidar-serial> # sets 192.168.1.5/24, pings the lidar
python3 tools/set_lidar_ip.py <lidar-serial>     # writes 192.168.1.1XX into MID360_config.json
```

### 5. Quick checks before Docker (no hardware)
```bash
PYTHONPATH=src/elev_bringup python3 -m unittest tools/test_core.py
PYTHONPATH=src/elev_bringup python3 tools/offline_test.py cpu
PYTHONPATH=src/elev_bringup python3 tools/offline_test.py cuda   # needs CuPy on the host; container is the normal path
```

### 6. Build and run the container
```bash
docker compose build             # slow the first time (ZED SDK, Livox SDK2, colcon)
docker compose up                # runs full.launch.py (lidar + ZED + TF + mapper)
```
Shell into the running container from another terminal:
```bash
docker compose exec elev bash
ros2 topic hz /livox/lidar       # expect ~10 Hz
ros2 run tf2_ros tf2_echo odom base_link
scripts/diagnose.sh
```

### 7. Day-to-day work
- Edit on the Jetson, or on the laptop and `git push` / `git pull` on the Jetson.
- Python changes need no rebuild (`--symlink-install` and the compose volume). Restart the launch.
- Dockerfile changes need `docker compose build` again.
- Commit calibrated values in `mount.yaml`; they are specific to this robot.

### Known first-build risks
1. Base image tag or ZED SDK URL does not match the L4T minor (404 on download).
2. `cupy-cuda11x` on Python 3.8 arm64 may compile from source (slow) or fail.
3. A missing ROS package: `rosdep` covers what it can; add any remaining one to the Dockerfile.
4. ZED wrapper (`humble-v4.2.x`) needs ZED SDK 4.2. Keep both in sync.

If the build fails, send the last ~30 lines of `docker compose build` output.

## Run (without the full setup)
0. Offline logic test (no ROS, no GPU): `PYTHONPATH=src/elev_bringup python3 tools/offline_test.py cpu`
1. Simulation (no hardware): `ros2 launch elev_bringup sim.launch.py backend:=cpu`
2. Real robot, after the setup above and `VERIFY.md` steps 0–7:
   - `ros2 launch elev_bringup full.launch.py backend:=cuda`
   - `ros2 launch elev_bringup mapping.launch.py rviz:=true backend:=cuda` (if sensors already run)
   - rviz on a laptop: same `ROS_DOMAIN_ID` (0) and `config/cyclonedds.xml` (`CYCLONEDDS_URI`).

## Mount calibration
```bash
ros2 launch elev_bringup sensors.launch.py          # robot on flat ground
ros2 run elev_bringup check_floor                   # prints tilt and a suggested mount
python3 tools/set_mount.py base_to_livox z=0.40 roll=0.0 pitch=0.7854   # apply, then relaunch
```
Repeat until the tilt is about 0 deg.

## Tuning (config/mapper.yaml)
resolution/size_m, measure (max|mean), z window, self_filter box (set to your robot body),
var_base/range_var_coeff (sensor noise), process_var (VIO drift), gate_sigma (reset sensitivity).
