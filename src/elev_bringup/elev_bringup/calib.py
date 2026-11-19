"""ROS-free mount calibration math (testable offline)."""
import numpy as np


def fit_floor(p, low_fraction=0.3):
    """p: (N,3) points in base_link. Returns the floor's tilt and height.
    rise_x_deg / rise_y_deg: degrees the floor climbs toward +x / +y (0 when flat).
    floor_z: height of the floor at base_link's origin (0 when base_link sits on the floor)."""
    z_cut = np.percentile(p[:, 2], 100 * low_fraction)
    floor = p[p[:, 2] < z_cut]
    c = floor.mean(axis=0)
    n = np.linalg.svd(floor - c)[2][2]
    n = n if n[2] > 0 else -n
    return {
        'n_floor': len(floor),
        'tilt_deg': float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0)))),
        'rise_x_deg': float(np.degrees(np.arctan2(-n[0], n[2]))),
        'rise_y_deg': float(np.degrees(np.arctan2(-n[1], n[2]))),
        'floor_z': float(c[2] + (n[0] * c[0] + n[1] * c[1]) / n[2]),
    }


def suggest_mount(mount, fit):
    """Corrected base_to_livox values that should flatten the floor (roll/pitch in rad, z in m)."""
    return {
        'z': mount['z'] - fit['floor_z'],
        'roll': mount['roll'] - np.radians(fit['rise_y_deg']),
        'pitch': mount['pitch'] + np.radians(fit['rise_x_deg']),
    }
