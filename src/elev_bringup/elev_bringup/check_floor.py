"""Mount calibration helper: stand on flat ground, run this, apply the printed correction to
config/mount.yaml (tools/set_mount.py does that), relaunch and repeat until the floor is flat."""
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from .calib import fit_floor, suggest_mount
from .tf_math import rpy_to_R


class FloorCheck(Node):
    def __init__(self):
        super().__init__('check_floor')
        path = os.path.join(get_package_share_directory('elev_bringup'), 'config', 'mount.yaml')
        with open(path) as f:
            self.mount = yaml.safe_load(f)['base_to_livox']
        m = self.mount
        self.R = rpy_to_R(m['roll'], m['pitch'], m['yaw'])
        self.t = np.array([m['x'], m['y'], m['z']])
        self.n = 0
        self.create_subscription(PointCloud2, '/livox/lidar', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.n += 1
        if self.n % 10:
            return
        a = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.column_stack([a['x'], a['y'], a['z']]).astype(np.float64) @ self.R.T + self.t
        p = p[np.hypot(p[:, 0], p[:, 1]) < 5.0]
        if len(p) < 500:
            return
        fit = fit_floor(p)
        new = suggest_mount(self.mount, fit)
        self.get_logger().info(
            f"floor tilt {fit['tilt_deg']:.2f} deg (rises toward +x {fit['rise_x_deg']:+.2f}, "
            f"toward +y {fit['rise_y_deg']:+.2f}), floor z at base origin {fit['floor_z']:+.3f} m, "
            f"{fit['n_floor']} pts. Suggest: z={new['z']:.4f} roll={new['roll']:.4f} "
            f"pitch={new['pitch']:.4f} (rad)")


def main():
    rclpy.init()
    rclpy.spin(FloorCheck())
