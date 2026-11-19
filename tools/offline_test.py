"""Run without ROS/GPU:  PYTHONPATH=src/elev_bringup python3 tools/offline_test.py [cpu|cuda]
Drives a virtual robot through sim_world, runs the real mapper pipeline, checks the result."""
import sys
import numpy as np
from elev_bringup.elevation_core import ElevationGrid, MapperConfig, get_backend, process_cloud, to_host
from elev_bringup.sim_world import sample_world, view_sensor, RAMP_SLOPE_DEG
from elev_bringup.tf_math import rpy_to_R

xp = get_backend(sys.argv[1] if len(sys.argv) > 1 else 'cpu')
cfg = MapperConfig(); grid = ElevationGrid(xp, cfg)
T_bs = np.eye(4); T_bs[:3, :3] = rpy_to_R(0, np.radians(45), 0); T_bs[:3, 3] = [0.2, 0, 0.1]
rs = np.random.default_rng(1)
for i in range(205):
    rx = min(0.04 * i, 7.0)
    T_ob = np.eye(4); T_ob[:3, 3] = [rx, 0, 0.4]
    process_cloud(grid, xp.asarray(view_sensor(sample_world(rs, rx), T_ob, T_bs, rs)), T_bs, T_ob, cfg)
x, y, z, sl, v = to_host(grid.to_points()).T
floor = (x > .5) & (x < 2.8) & (abs(y) > 1.5) & (abs(y) < 4)
top = (x > 3.2) & (x < 3.8) & (y > .2) & (y < .8)
ramp = (x > 6.3) & (x < 8.6) & (abs(y) < 1.5)
res = {'floor z ~0': abs(z[floor].mean()) < 0.03, 'box top ~0.5': abs(z[top].mean() - 0.5) < 0.05,
       'ramp slope': abs(sl[ramp].mean() - RAMP_SLOPE_DEG) < 2.0}
print(res); sys.exit(0 if all(res.values()) else 1)
