import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

class MimicPerception(Node):
    """
    Publishes /local_cones as geometry_msgs/PoseArray at 10 Hz.
    - header.stamp = detection time
    - header.frame_id = "base_link"
    - Each Pose.position.{x,y} = cone in local frame
    - TEMP: Pose.position.z encodes isTall (1.0 tall, 0.0 short)
    """

    def __init__(self):
        super().__init__('mimic_perception')
        self.pub = self.create_publisher(PoseArray, '/local_cones', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/vehicle_pose', self._pose_cb, 10)
        self.current_pose: PoseStamped | None = None

        # Simple global cones in MAP frame: (x, y, is_tall)
        self.global_cones_map = [
            (5.0, 0.0, False),
            (5.0, 3.0, True),
            (10.0, 1.0, False),
            (12.0, -2.0, True),
            (8.0, -3.0, False)
        ]

        # Parameters (can override via launch)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('range_x_max', 25.0)
        self.declare_parameter('range_y_abs_max', 10.0)
        self.declare_parameter('max_cones_per_msg', 50)

        self.rate = float(self.get_parameter('publish_rate_hz').value)
        self.rng_x = float(self.get_parameter('range_x_max').value)
        self.rng_y = float(self.get_parameter('range_y_abs_max').value)
        self.max_n = int(self.get_parameter('max_cones_per_msg').value)

        self.timer = self.create_timer(1.0 / max(self.rate, 1e-6), self._tick)
        self.get_logger().info('mimic_perception: publishing /local_cones (PoseArray) at %.1f Hz' % self.rate)

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _tick(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # REQUIRED by your spec

        if self.current_pose is None:
            # Still publish empty [] at 10 Hz so downstream knows we are alive
            self.pub.publish(msg)
            return

        # Vehicle pose in MAP frame
        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        q = self.current_pose.pose.orientation
        # quat -> yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Map all global cones into base_link; gate by range; cap count
        count = 0
        for cx, cy, is_tall in self.global_cones_map:
            dx, dy = cx - px, cy - py
            lx =  dx * math.cos(yaw) + dy * math.sin(yaw)
            ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
            if lx < 0.0 or lx > self.rng_x or abs(ly) > self.rng_y:
                continue

            p = Pose()
            p.position.x = float(lx)
            p.position.y = float(ly)
            # TEMP shim for isTall (since we aren't using a custom msg yet)
            p.position.z = 1.0 if is_tall else 0.0
            p.orientation.w = 1.0  # identity; orientation unused for cones
            msg.poses.append(p)

            count += 1
            if count >= self.max_n:
                break

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MimicPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
