"""ROS-free unit tests. Run:  PYTHONPATH=src/elev_bringup python3 -m unittest tools/test_core.py
Covers TF math, the calibration correction (must converge to the true mount), the mount.yaml
writer, and the elevation grid's rolling window and reset gate."""
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from elev_bringup.calib import fit_floor, suggest_mount  # noqa: E402
from elev_bringup.elevation_core import ElevationGrid, MapperConfig, get_backend  # noqa: E402
from elev_bringup.tf_math import invert, rpy_to_R, R_to_rpy  # noqa: E402
from set_mount import set_mount  # noqa: E402


class TfMath(unittest.TestCase):
    def test_rpy_round_trip(self):
        r, p, y = 0.1, 0.7853981634, -0.3
        self.assertTrue(np.allclose(R_to_rpy(rpy_to_R(r, p, y)), (r, p, y), atol=1e-9))

    def test_invert_composes_to_identity(self):
        x, y, z, r, p, yw = 0.1, -0.2, 0.3, 0.05, 0.4, 0.2
        ix, iy, iz, ir, ip, iyw = invert(x, y, z, r, p, yw)
        T = np.eye(4); T[:3, :3] = rpy_to_R(r, p, yw); T[:3, 3] = [x, y, z]
        Ti = np.eye(4); Ti[:3, :3] = rpy_to_R(ir, ip, iyw); Ti[:3, 3] = [ix, iy, iz]
        self.assertTrue(np.allclose(T @ Ti, np.eye(4), atol=1e-9))


class Calibration(unittest.TestCase):
    def test_correction_converges_to_true_mount(self):
        rs = np.random.default_rng(0)
        true_R, true_t = rpy_to_R(np.radians(1.5), np.radians(41.0), 0.0), np.array([0.1, 0.0, 0.4])
        fx, fy = rs.uniform(-4, 4, 4000), rs.uniform(-4, 4, 4000)
        ps = (np.stack([fx, fy, np.zeros_like(fx)], 1) - true_t) @ true_R   # flat floor, sensor frame
        m = dict(x=0.0, y=0.0, z=0.30, roll=0.0, pitch=np.radians(45.0), yaw=0.0)
        for _ in range(4):
            R = rpy_to_R(m['roll'], m['pitch'], m['yaw'])
            fit = fit_floor(ps @ R.T + np.array([0.0, 0.0, m['z']]))
            m.update(suggest_mount(m, fit))
        self.assertAlmostEqual(np.degrees(m['pitch']), 41.0, places=1)
        self.assertAlmostEqual(np.degrees(m['roll']), 1.5, places=1)
        self.assertAlmostEqual(m['z'], 0.40, places=2)


class MountWriter(unittest.TestCase):
    TEXT = ("# header\nbase_to_livox:\n  x: 0.0\n  z: 0.0   # height\n  pitch: 0.7853981634   # 45 deg\n"
            "\nbase_to_zed:\n  x: 0.0\n")

    def test_updates_block_and_keeps_comments(self):
        out = set_mount(self.TEXT, 'base_to_livox', {'z': 0.4, 'pitch': 0.5})
        self.assertIn('  z: 0.400000   # height', out)
        self.assertIn('  pitch: 0.500000   # 45 deg', out)
        self.assertIn('\n\nbase_to_zed:\n  x: 0.0\n', out)     # other block untouched

    def test_unknown_key_fails(self):
        with self.assertRaises(SystemExit):
            set_mount(self.TEXT, 'base_to_livox', {'nope': 1.0})


class GridBehaviour(unittest.TestCase):
    def test_window_follows_robot_and_keeps_world_cells(self):
        cfg = MapperConfig(resolution=0.1, size_m=4.0)
        g = ElevationGrid(get_backend('cpu'), cfg)
        g.recenter(0.0, 0.0)
        g.elev[:] = np.nan
        i, j = g.n // 2 + 5, g.n // 2 - 3
        g.elev[i, j] = 1.23
        world_x, world_y = (i + g.origin[0] + 0.5) * cfg.resolution, (j + g.origin[1] + 0.5) * cfg.resolution
        g.recenter(0.5, -0.3)
        ii = int(np.floor(world_x / cfg.resolution)) - g.origin[0]
        jj = int(np.floor(world_y / cfg.resolution)) - g.origin[1]
        self.assertAlmostEqual(float(g.elev[ii, jj]), 1.23, places=5)

    def test_reset_gate_replaces_disagreeing_cell(self):
        cfg = MapperConfig(resolution=0.1, size_m=4.0)
        xp = get_backend('cpu')
        g = ElevationGrid(xp, cfg)
        g.recenter(0.0, 0.0)
        pts = np.array([[0.05, 0.05, 0.0]] * 50, dtype=np.float32)
        rng = np.full(50, 3.0, dtype=np.float32)
        for _ in range(20):
            g.integrate(pts, rng)
        self.assertAlmostEqual(float(np.nanmax(g.elev)), 0.0, places=3)
        g.integrate(np.array([[0.05, 0.05, 1.0]] * 50, dtype=np.float32), rng)
        self.assertAlmostEqual(float(np.nanmax(g.elev)), 1.0, places=3)


if __name__ == '__main__':
    unittest.main()
