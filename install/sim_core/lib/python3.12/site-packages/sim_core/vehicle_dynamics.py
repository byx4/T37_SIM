import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

def clamp(x, lo, hi): return max(lo, min(hi, x))

class VehicleDynamics(Node):
    """
    Kinematic bicycle model (no slip):
      x_dot   = v * cos(psi)
      y_dot   = v * sin(psi)
      psi_dot = v / L * tan(delta)

    Inputs via /control_action (geometry_msgs/Twist):
      - throttle = msg.linear.x in [0,1] -> target speed v_target = v_max * throttle
      - steering = msg.angular.z in [0,1] -> wheel angle delta in [-delta_max, +delta_max]
        with 0.5 = straight
    Publishes /vehicle_pose (PoseStamped, frame='map')
    """

    def __init__(self):
        super().__init__('vehicle_dynamics')

        # Parameters (tunable)
        self.declare_parameter('dt', 0.02)              # 50 Hz internal integration
        self.declare_parameter('L', 1.6)                # wheelbase (m)
        self.declare_parameter('v_max', 8.0)            # max speed for throttle mapping (m/s)
        self.declare_parameter('accel_gain', 2.5)       # speed response gain
        self.declare_parameter('delta_max_deg', 28.0)   # max front wheel angle (deg)
        self.declare_parameter('cmd_timeout', 0.25)     # seconds; stop if stale
        self.declare_parameter('yaw_rate_limit', 3.0)   # rad/s clamp for stability

        self.dt              = float(self.get_parameter('dt').value)
        self.L               = float(self.get_parameter('L').value)
        self.v_max           = float(self.get_parameter('v_max').value)
        self.accel_gain      = float(self.get_parameter('accel_gain').value)
        self.delta_max       = math.radians(float(self.get_parameter('delta_max_deg').value))
        self.cmd_timeout     = float(self.get_parameter('cmd_timeout').value)
        self.yaw_rate_limit  = float(self.get_parameter('yaw_rate_limit').value)

        # State
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        self.v = 0.0

        # Command (normalized)
        self.throttle = 0.0   # [0,1]
        self.steering = 0.5   # [0,1], center
        self.last_cmd_time = self.get_clock().now()

        # I/O
        self.pose_pub = self.create_publisher(PoseStamped, '/vehicle_pose', 10)
        # ControlAction shim: geometry_msgs/Twist (linear.x=throttle, angular.z=steering)
        self.create_subscription(Twist, '/control_action', self._cmd_cb, 10)

        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info('vehicle_dynamics: kinematic bicycle model running')

    def _cmd_cb(self, msg: Twist):
        self.throttle = clamp(float(msg.linear.x), 0.0, 1.0)
        self.steering = clamp(float(msg.angular.z), 0.0, 1.0)
        self.last_cmd_time = self.get_clock().now()

    def _step(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9

        # Timeout -> safe stop
        if age > self.cmd_timeout:
            v_target = 0.0
            steer_norm = 0.5
        else:
            v_target = self.v_max * self.throttle
            steer_norm = self.steering

        # Map steering to wheel angle delta in [-delta_max, +delta_max] (0.5 = straight)
        delta = (steer_norm - 0.5) * 2.0 * self.delta_max

        # First-order speed response towards v_target
        self.v += self.dt * self.accel_gain * (v_target - self.v)

        # Bicycle yaw rate
        psi_dot = 0.0 if abs(self.L) < 1e-6 else (self.v / self.L) * math.tan(delta)
        psi_dot = clamp(psi_dot, -self.yaw_rate_limit, self.yaw_rate_limit)

        # Integrate
        self.psi += psi_dot * self.dt
        self.psi = math.atan2(math.sin(self.psi), math.cos(self.psi))  # wrap
        self.x += self.v * math.cos(self.psi) * self.dt
        self.y += self.v * math.sin(self.psi) * self.dt

        # Publish pose (map)
        out = PoseStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'map'
        out.pose.position.x = self.x
        out.pose.position.y = self.y
        out.pose.position.z = 0.0
        out.pose.orientation.x = 0.0
        out.pose.orientation.y = 0.0
        out.pose.orientation.z = math.sin(self.psi * 0.5)
        out.pose.orientation.w = math.cos(self.psi * 0.5)
        self.pose_pub.publish(out)

def main():
    rclpy.init()
    node = VehicleDynamics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
