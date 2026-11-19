"""Synthetic world + Mid-360-like sampling, used by the fake lidar node and offline tests.
World (odom frame, floor z=0): flat floor, a 1x1x0.5 m box at x in [3,4], a ramp x in [6,9] rising 0.3 m/m."""
import numpy as np

BOX = dict(x0=3.0, x1=4.0, y0=0.0, y1=1.0, h=0.5)
RAMP = dict(x0=6.0, x1=9.0, y0=-2.0, y1=2.0, slope=0.3)
RAMP_SLOPE_DEG = float(np.degrees(np.arctan(RAMP['slope'])))


def sample_world(rs, rx, n=20000):
    fx = rs.uniform(rx - 1.0, rx + 12.0, n)
    fy = rs.uniform(-6.0, 6.0, n)
    inbox = (fx > BOX['x0']) & (fx < BOX['x1']) & (fy > BOX['y0']) & (fy < BOX['y1'])
    inramp = (fx > RAMP['x0']) & (fx < RAMP['x1']) & (fy > RAMP['y0']) & (fy < RAMP['y1'])
    keep = ~(inbox | inramp)
    floor = np.stack([fx[keep], fy[keep], np.zeros(keep.sum())], 1)
    m = n // 10
    bx = rs.uniform(BOX['x0'], BOX['x1'], m); by = rs.uniform(BOX['y0'], BOX['y1'], m)
    top = np.stack([bx, by, np.full(m, BOX['h'])], 1)
    fy2 = rs.uniform(BOX['y0'], BOX['y1'], m); fz = rs.uniform(0, BOX['h'], m)
    face = np.stack([np.full(m, BOX['x0']), fy2, fz], 1)
    rxp = rs.uniform(RAMP['x0'], RAMP['x1'], m); ryp = rs.uniform(RAMP['y0'], RAMP['y1'], m)
    ramp = np.stack([rxp, ryp, (rxp - RAMP['x0']) * RAMP['slope']], 1)
    return np.concatenate([floor, top, face, ramp]).astype(np.float64)


def view_sensor(world_pts, T_ob, T_bs, rs, fov=(-7.0, 52.0), rmin=0.3, rmax=12.0, noise=0.01):
    """World points -> sensor frame, filtered by Mid-360 vertical FOV (relative to the sensor plane)."""
    T_os = T_ob @ T_bs
    R, t = T_os[:3, :3], T_os[:3, 3]
    ps = (world_pts - t) @ R
    r = np.linalg.norm(ps, axis=1)
    elev = np.degrees(np.arcsin(np.clip(ps[:, 2] / np.maximum(r, 1e-9), -1, 1)))
    k = (elev >= fov[0]) & (elev <= fov[1]) & (r >= rmin) & (r <= rmax)
    ps = ps[k]
    return (ps + rs.normal(0, noise, ps.shape)).astype(np.float32)
