"""ROS2 node: Livox cloud -> GPU elevation grid -> PointCloud2 for rviz2."""
import time
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from .elevation_core import ElevationGrid, MapperConfig, get_backend, process_cloud, to_host


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_to_matrix(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def cloud_xyz(msg):
    """Zero-parse extraction of x,y,z (float32) from a PointCloud2 with arbitrary point_step."""
    n = msg.width * msg.height
    if n == 0:
        return None
    offs = {f.name: f.offset for f in msg.fields if f.datatype == PointField.FLOAT32}
    if not all(a in offs for a in 'xyz'):
        return None
    step = msg.point_step
    cols = [np.ndarray((n,), dtype='<f4', buffer=msg.data, offset=offs[a], strides=(step,))
            for a in 'xyz']
    return np.stack(cols, axis=1)


class ElevationMapper(Node):
    def __init__(self):
        super().__init__('elevation_mapper')
        d = self.declare_parameter
        d('backend', 'auto'); d('cloud_topic', '/livox/lidar')
        d('fixed_frame', 'odom'); d('base_frame', 'base_link')
        d('resolution', 0.10); d('size_m', 12.0); d('measure', 'max')
        d('min_range', 0.3); d('max_range', 10.0)
        d('z_min_rel', -1.5); d('z_max_rel', 3.0)
        d('self_filter', [0.0] * 6)
        d('var_base', 4e-4); d('range_var_coeff', 4e-3); d('process_var', 1e-5)
        d('forget_var', 0.25); d('gate_sigma', 3.0)
        d('publish_hz', 5.0); d('use_msg_stamp', False); d('tf_timeout', 0.05)

        g = lambda k: self.get_parameter(k).value
        self.cfg = MapperConfig(
            resolution=g('resolution'), size_m=g('size_m'), measure=g('measure'),
            min_range=g('min_range'), max_range=g('max_range'),
            z_min_rel=g('z_min_rel'), z_max_rel=g('z_max_rel'),
            self_filter=list(g('self_filter')), var_base=g('var_base'),
            range_var_coeff=g('range_var_coeff'), process_var=g('process_var'),
            forget_var=g('forget_var'), gate_sigma=g('gate_sigma'))
        self.xp = get_backend(g('backend'))
        self.grid = ElevationGrid(self.xp, self.cfg)
        self.fixed, self.base = g('fixed_frame'), g('base_frame')
        self.use_msg_stamp, self.tf_timeout = g('use_msg_stamp'), g('tf_timeout')
        self.get_logger().info(
            f'backend={"CUDA (CuPy)" if self.xp is not np else "CPU (NumPy)"} '
            f'grid={self.grid.n}x{self.grid.n} @ {self.cfg.resolution} m')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, '/elevation_map/cloud', 1)
        self.create_timer(1.0 / g('publish_hz'), self.publish)
        self.create_timer(5.0, self.report)
        self.times, self.frames = [], 0

    def _lookup(self, target, source, stamp):
        return tf_to_matrix(self.tf_buffer.lookup_transform(
            target, source, stamp, timeout=Duration(seconds=self.tf_timeout)))

    def on_cloud(self, msg):
        t0 = time.perf_counter()
        stamp = Time.from_msg(msg.header.stamp) if self.use_msg_stamp else Time()
        try:
            T_bs = self._lookup(self.base, msg.header.frame_id, stamp)
            T_ob = self._lookup(self.fixed, self.base, stamp)
        except TransformException as e:
            self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=2.0)
            return
        xyz = cloud_xyz(msg)
        if xyz is None:
            self.get_logger().error('cloud has no float32 x,y,z', throttle_duration_sec=5.0)
            return
        process_cloud(self.grid, self.xp.asarray(xyz), T_bs, T_ob, self.cfg)
        self.frames += 1
        self.times.append(time.perf_counter() - t0)

    def publish(self):
        if self.grid.origin is None:
            return
        pts = to_host(self.grid.to_points())
        m = PointCloud2()
        m.header.frame_id = self.fixed
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = 1, int(pts.shape[0])
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(['x', 'y', 'z', 'intensity', 'variance'])]
        m.is_bigendian, m.point_step, m.is_dense = False, 20, True
        m.row_step = 20 * m.width
        m.data = np.ascontiguousarray(pts, dtype='<f4').tobytes()
        self.pub.publish(m)

    def report(self):
        if self.times:
            t = np.array(self.times) * 1e3
            self.get_logger().info(
                f'{self.frames} frames, process mean {t.mean():.1f} ms, max {t.max():.1f} ms')
            self.times = []


def main():
    rclpy.init()
    rclpy.spin(ElevationMapper())
