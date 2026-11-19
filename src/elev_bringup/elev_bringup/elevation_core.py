"""Backend-agnostic (CuPy / NumPy) 2.5D elevation grid. No ROS imports: unit-testable offline.

Map lives in the fixed frame (odom), is robot-centred and rolling. Layers: elevation, variance.
Per frame: bin points into cells (GPU scatter ops), take per-cell max (or mean) height as the
measurement, then Kalman-fuse with the stored height. Large disagreement (> gate_sigma) resets
the cell, which handles moved obstacles and VIO drift.
"""
from dataclasses import dataclass, field
import numpy as np

try:  # CUDA backend is optional so the same code can be tested on a laptop
    import cupy as cp
    import cupyx
    _HAS_CUPY = True
except Exception:  # pragma: no cover
    cp = None
    cupyx = None
    _HAS_CUPY = False


def get_backend(name='auto'):
    if name == 'cpu':
        return np
    if name in ('cuda', 'auto'):
        ok = _HAS_CUPY
        if ok:
            try:
                ok = cp.cuda.runtime.getDeviceCount() > 0
            except Exception:
                ok = False
        if ok:
            return cp
        if name == 'cuda':
            raise RuntimeError('backend=cuda requested but CuPy/CUDA device is not available')
        return np
    raise ValueError(f'unknown backend {name}')


def to_host(a):
    return a.get() if hasattr(a, 'get') else np.asarray(a)


@dataclass
class MapperConfig:
    resolution: float = 0.10
    size_m: float = 12.0
    measure: str = 'max'            # 'max' (obstacle-friendly) or 'mean'
    min_range: float = 0.3
    max_range: float = 10.0
    z_min_rel: float = -1.5         # height window relative to robot z (fixed frame)
    z_max_rel: float = 3.0
    self_filter: list = field(default_factory=lambda: [0.0] * 6)  # base_link box xmin,xmax,ymin,ymax,zmin,zmax
    var_base: float = 4e-4          # (2 cm)^2
    range_var_coeff: float = 4e-3   # extra sigma per metre of range (pose/angle error)
    process_var: float = 1e-5       # added per integration to every valid cell (drift)
    forget_var: float = 0.25
    gate_sigma: float = 3.0

    @property
    def self_filter_enabled(self):
        f = self.self_filter
        return len(f) == 6 and (f[0] != f[1] or f[2] != f[3] or f[4] != f[5])


class ElevationGrid:
    def __init__(self, xp, cfg: MapperConfig):
        self.xp = xp
        self.cfg = cfg
        n = int(round(cfg.size_m / cfg.resolution))
        self.n = n + (n % 2)
        self.res = cfg.resolution
        self.origin = None  # integer cell index (ox, oy) of array element [0, 0]
        self.elev = xp.full((self.n, self.n), np.nan, dtype=xp.float32)
        self.var = xp.zeros((self.n, self.n), dtype=xp.float32)

    # ---- scatter helpers -------------------------------------------------
    def _scatter_add(self, target, idx, vals):
        if self.xp is np:
            np.add.at(target, idx, vals)
        else:
            cupyx.scatter_add(target, idx, vals)

    def _scatter_max(self, target, idx, vals):
        if self.xp is np:
            np.maximum.at(target, idx, vals)
        else:
            cupyx.scatter_max(target, idx, vals)

    # ---- rolling window --------------------------------------------------
    def recenter(self, x, y):
        xp, n, res = self.xp, self.n, self.res
        nox = int(np.floor(x / res)) - n // 2
        noy = int(np.floor(y / res)) - n // 2
        if self.origin is None:
            self.origin = (nox, noy)
            return
        dx, dy = nox - self.origin[0], noy - self.origin[1]
        if dx == 0 and dy == 0:
            return
        if abs(dx) >= n or abs(dy) >= n:
            self.elev.fill(np.nan)
        else:
            for name in ('elev', 'var'):
                a = getattr(self, name)
                a = xp.roll(a, -dx, axis=0)
                a = xp.roll(a, -dy, axis=1)
                setattr(self, name, a)
            if dx > 0:
                self.elev[n - dx:, :] = np.nan
            elif dx < 0:
                self.elev[:-dx, :] = np.nan
            if dy > 0:
                self.elev[:, n - dy:] = np.nan
            elif dy < 0:
                self.elev[:, :-dy] = np.nan
        self.origin = (nox, noy)

    # ---- measurement update ----------------------------------------------
    def integrate(self, pts, rng):
        """pts: (N,3) float32 in the fixed frame; rng: (N,) sensor range of each point."""
        xp, n, c = self.xp, self.n, self.cfg
        ix = xp.floor(pts[:, 0] / self.res).astype(xp.int32) - self.origin[0]
        iy = xp.floor(pts[:, 1] / self.res).astype(xp.int32) - self.origin[1]
        m = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        ix, iy, z, r = ix[m], iy[m], pts[:, 2][m], rng[m]
        used = int(ix.size)
        if used == 0:
            return 0
        flat = ix * n + iy
        nn = n * n
        zmax = xp.full(nn, -np.inf, dtype=xp.float32)
        cnt = xp.zeros(nn, dtype=xp.float32)
        zsum = xp.zeros(nn, dtype=xp.float32)
        rsum = xp.zeros(nn, dtype=xp.float32)
        self._scatter_max(zmax, flat, z)
        self._scatter_add(cnt, flat, xp.ones_like(z))
        self._scatter_add(zsum, flat, z)
        self._scatter_add(rsum, flat, r)

        has = (cnt > 0).reshape(n, n)
        safe = xp.maximum(cnt, 1.0)
        zm = (zmax if c.measure == 'max' else zsum / safe).reshape(n, n)
        zm = xp.where(has, zm, 0.0).astype(xp.float32)
        rm = (rsum / safe).reshape(n, n)
        varm = (c.var_base + (c.range_var_coeff * rm) ** 2).astype(xp.float32)
        if c.measure != 'max':
            varm = varm / safe.reshape(n, n)

        h, v = self.elev, self.var
        valid = ~xp.isnan(h)
        v = xp.where(valid, v + c.process_var, v)           # drift / ageing
        upd = has & valid
        new = has & ~valid
        d2 = (zm - h) ** 2 / (v + varm)
        reset = upd & (d2 > c.gate_sigma ** 2)
        fuse = upd & ~reset
        k = v / (v + varm)
        h = xp.where(fuse, h + k * (zm - h), h)
        v = xp.where(fuse, v * varm / (v + varm), v)
        setm = new | reset
        h = xp.where(setm, zm, h)
        v = xp.where(setm, varm, v)
        h = xp.where(v > c.forget_var, xp.float32(np.nan), h)  # forget stale cells
        self.elev, self.var = h.astype(xp.float32), v.astype(xp.float32)
        return used

    # ---- output ----------------------------------------------------------
    def to_points(self):
        """(M,5) float32 on the backend: x, y, z, slope_deg, variance for every valid cell."""
        xp, res = self.xp, self.res
        h = self.elev
        valid = ~xp.isnan(h)
        ii, jj = xp.nonzero(valid)
        x = (ii + self.origin[0] + 0.5) * res
        y = (jj + self.origin[1] + 0.5) * res
        gx, gy = xp.gradient(h, res)
        slope = xp.degrees(xp.arctan(xp.hypot(gx, gy)))
        slope = xp.nan_to_num(slope)[ii, jj]
        out = xp.stack([x, y, h[ii, jj], slope, self.var[ii, jj]], axis=1)
        return out.astype(xp.float32)


def process_cloud(grid, pts_sensor, T_bs, T_ob, cfg: MapperConfig):
    """Full per-frame pipeline. pts_sensor: (N,3) on the backend (sensor frame).
    T_bs: base_link <- sensor 4x4 (numpy). T_ob: fixed_frame <- base_link 4x4 (numpy)."""
    xp = grid.xp
    pts = pts_sensor[xp.isfinite(pts_sensor).all(axis=1)]
    rng = xp.sqrt((pts * pts).sum(axis=1))
    m = (rng >= cfg.min_range) & (rng <= cfg.max_range)
    pts, rng = pts[m], rng[m]

    R_bs = xp.asarray(T_bs[:3, :3], dtype=xp.float32)
    t_bs = xp.asarray(T_bs[:3, 3], dtype=xp.float32)
    R_ob = xp.asarray(T_ob[:3, :3], dtype=xp.float32)
    t_ob = xp.asarray(T_ob[:3, 3], dtype=xp.float32)

    pb = pts @ R_bs.T + t_bs
    if cfg.self_filter_enabled:
        x0, x1, y0, y1, z0, z1 = cfg.self_filter
        inside = ((pb[:, 0] >= x0) & (pb[:, 0] <= x1) & (pb[:, 1] >= y0) &
                  (pb[:, 1] <= y1) & (pb[:, 2] >= z0) & (pb[:, 2] <= z1))
        pb, rng = pb[~inside], rng[~inside]
    po = pb @ R_ob.T + t_ob
    zrel = po[:, 2] - float(T_ob[2, 3])
    k = (zrel >= cfg.z_min_rel) & (zrel <= cfg.z_max_rel)
    po, rng = po[k], rng[k]

    grid.recenter(float(T_ob[0, 3]), float(T_ob[1, 3]))
    return grid.integrate(po, rng)
