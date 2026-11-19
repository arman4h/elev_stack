"""Hardware-free test source: drives a virtual robot through sim_world and publishes
/livox/lidar (frame livox_frame) + TF odom->base_link. base_link->livox_frame must come from
the launch file (static TF from mount.yaml)."""
import os
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

from .sim_world import sample_world, view_sensor
from .tf_math import rpy_to_R


class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        path = os.path.join(get_package_share_directory('elev_bringup'), 'config', 'mount.yaml')
        m = yaml.safe_load(open(path))['base_to_livox']
        self.T_bs = np.eye(4)
        self.T_bs[:3, :3] = rpy_to_R(m['roll'], m['pitch'], m['yaw'])
        self.T_bs[:3, 3] = [m['x'], m['y'], m['z']]
        self.rs = np.random.default_rng(0)
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(PointCloud2, '/livox/lidar', 5)
        self.t0 = self.get_clock().now()
        self.create_timer(0.1, self.tick)

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        rx = min(0.4 * t, 7.0)               # drive forward, then stop
        T_ob = np.eye(4); T_ob[:3, 3] = [rx, 0.0, 0.4]   # base_link 0.4 m above the floor
        tf = TransformStamped()
        tf.header.stamp = now.to_msg(); tf.header.frame_id = 'odom'; tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.z = rx, 0.4
        tf.transform.rotation.w = 1.0
        self.br.sendTransform(tf)
        pts = view_sensor(sample_world(self.rs, rx), T_ob, self.T_bs, self.rs)
        msg = PointCloud2()
        msg.header.stamp = now.to_msg(); msg.header.frame_id = 'livox_frame'
        msg.height, msg.width = 1, int(pts.shape[0])
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 12, True
        msg.row_step = 12 * msg.width
        msg.data = np.ascontiguousarray(pts, dtype='<f4').tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeLidar())
