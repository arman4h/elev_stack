"""ROS-free tests for the elevation grid.
Run from src/elev_bringup:  python3 -m unittest discover -s test -v"""
import unittest

import numpy as np

from elev_bringup.elevation_core import ElevationGrid, MapperConfig, get_backend


class GridBehaviour(unittest.TestCase):
    def make_grid(self):
        cfg = MapperConfig(resolution=0.1, size_m=4.0)
        g = ElevationGrid(get_backend('cpu'), cfg)
        g.recenter(0.0, 0.0)
        return g, cfg

    def test_window_follows_robot_and_keeps_world_cells(self):
        g, cfg = self.make_grid()
        g.elev[:] = np.nan
        i, j = g.n // 2 + 5, g.n // 2 - 3
        g.elev[i, j] = 1.23
        wx = (i + g.origin[0] + 0.5) * cfg.resolution
        wy = (j + g.origin[1] + 0.5) * cfg.resolution
        g.recenter(0.5, -0.3)
        ii = int(np.floor(wx / cfg.resolution)) - g.origin[0]
        jj = int(np.floor(wy / cfg.resolution)) - g.origin[1]
        self.assertAlmostEqual(float(g.elev[ii, jj]), 1.23, places=5)

    def test_reset_gate_replaces_disagreeing_cell(self):
        g, _ = self.make_grid()
        pts = np.array([[0.05, 0.05, 0.0]] * 50, dtype=np.float32)
        rng = np.full(50, 3.0, dtype=np.float32)
        for _ in range(20):
            g.integrate(pts, rng)
        self.assertAlmostEqual(float(np.nanmax(g.elev)), 0.0, places=3)
        g.integrate(np.array([[0.05, 0.05, 1.0]] * 50, dtype=np.float32), rng)
        self.assertAlmostEqual(float(np.nanmax(g.elev)), 1.0, places=3)


if __name__ == '__main__':
    unittest.main()
