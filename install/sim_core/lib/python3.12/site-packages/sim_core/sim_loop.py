import math
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Twist, TransformStamped
from tf2_ros import TransformBroadcaster

def clamp(x, lo, hi): return max(lo, min(hi, x))

class SimLoop(Node):
    """
    Single-process sim timestep:
      - Reads /control_action (Twist: throttle in linear.x, steering in angular.z)
      - Kinematic bicycle to update vehicle pose in 'map'
      - Every 10 Hz: publishes /local_cones (PoseArray in 'base_link') from YAML map
      - Publishes TF: map -> base_link
    """

    def __init__(self):
        super().__init__('sim_loop')

        # ---------------- Params ----------------
        # Paths / map
        self.declare_parameter('map_path', '')
        self.declare_parameter('map_frame_id', 'map')   # interpret YAML frame as this global frame
        # Vehicle / bicycle
        self.declare_parameter('dt', 0.02)              # 50 Hz physics
        self.declare_parameter('L', 1.6)                # wheelbase (m)
        self.declare_parameter('v_max', 8.0)            # throttle mapping (m/s)
        self.declare_parameter('accel_gain', 2.5)       # 1st-order speed response gain
        self.declare_parameter('delta_max_deg', 28.0)   # max wheel angle
        self.declare_parameter('cmd_timeout', 0.25)     # s (no cmd -> stop)
        self.declare_parameter('yaw_rate_limit', 3.0)   # rad/s clamp
        # Perception publishing
        self.declare_parameter('cones_rate_hz', 10.0)
        self.declare_parameter('range_x_max', 25.0)
        self.declare_parameter('range_y_abs_max', 10.0)
        self.declare_parameter('max_cones_per_msg', 50)

        self.map_path        = str(self.get_parameter('map_path').value)
        self.map_frame_id    = str(self.get_parameter('map_frame_id').value)
        self.dt              = float(self.get_parameter('dt').value)
        self.L               = float(self.get_parameter('L').value)
        self.v_max           = float(self.get_parameter('v_max').value)
        self.accel_gain      = float(self.get_parameter('accel_gain').value)
        self.delta_max       = math.radians(float(self.get_parameter('delta_max_deg').value))
        self.cmd_timeout     = float(self.get_parameter('cmd_timeout').value)
        self.yaw_rate_limit  = float(self.get_parameter('yaw_rate_limit').value)

        self.cones_rate_hz   = float(self.get_parameter('cones_rate_hz').value)
        self.range_x_max     = float(self.get_parameter('range_x_max').value)
        self.range_y_abs_max = float(self.get_parameter('range_y_abs_max').value)
        self.max_cones       = int(self.get_parameter('max_cones_per_msg').value)

        # ---------------- Load YAML map ----------------
        self.world_cones = []  # list of (x,y)
        if self.map_path and os.path.isfile(self.map_path):
            with open(self.map_path, 'r') as f:
                data = yaml.safe_load(f)
            # We accept frame_id but we don't transform frames; treat as map
            yaml_frame = data.get('frame_id', 'map')
            if yaml_frame != self.map_frame_id:
                self.get_logger().warn(f"YAML frame_id='{yaml_frame}' != param map_frame_id='{self.map_frame_id}'. "
                                       "Assuming same global frame for simplicity.")
            for c in data.get('cones', []):
                try:
                    self.world_cones.append((float(c['x']), float(c['y'])))
                except Exception:
                    continue
            self.get_logger().info(f"Loaded {len(self.world_cones)} cones from {self.map_path}")
        else:
            self.get_logger().warn("No valid map_path provided; using an empty cone map.")

        # ---------------- State ----------------
        self.x = 0.0; self.y = 0.0; self.psi = 0.0; self.v = 0.0
        self.throttle = 0.0  # [0,1]
        self.steering = 0.5  # [0,1], 0.5 = straight
        self.last_cmd_time = self.get_clock().now()
        self._cones_accum = 0.0  # to hit cones_rate_hz using dt accumulation

        # ---------------- I/O ----------------
        self.pose_pub = self.create_publisher(PoseStamped, '/vehicle_pose', 10)
        self.cones_pub = self.create_publisher(PoseArray, '/local_cones', 10)
        self.create_subscription(Twist, '/control_action', self._cmd_cb, 10)

        self.tf_pub = TransformBroadcaster(self)

        # ---------------- Timers ----------------
        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info("sim_loop running (bicycle + local_cones from YAML)")

    # ---------- Callbacks ----------
    def _cmd_cb(self, msg: Twist):
        self.throttle = clamp(float(msg.linear.x), 0.0, 1.0)
        self.steering = clamp(float(msg.angular.z), 0.0, 1.0)
        self.last_cmd_time = self.get_clock().now()

    # ---------- Core loop ----------
    def _step(self):
        now = self.get_clock().now()
        # 1) Controls → bicycle
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            v_target = 0.0
            steer_norm = 0.5
        else:
            v_target = self.v_max * self.throttle
            steer_norm = self.steering

        delta = (steer_norm - 0.5) * 2.0 * self.delta_max
        self.v += self.dt * self.accel_gain * (v_target - self.v)
        psi_dot = 0.0 if abs(self.L) < 1e-6 else (self.v / self.L) * math.tan(delta)
        psi_dot = clamp(psi_dot, -self.yaw_rate_limit, self.yaw_rate_limit)

        self.psi += psi_dot * self.dt
        self.psi = math.atan2(math.sin(self.psi), math.cos(self.psi))  # wrap
        self.x += self.v * math.cos(self.psi) * self.dt
        self.y += self.v * math.sin(self.psi) * self.dt

        # 2) Publish pose (map)
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.map_frame_id  # 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.z = math.sin(self.psi * 0.5)
        pose.pose.orientation.w = math.cos(self.psi * 0.5)
        self.pose_pub.publish(pose)

        # 3) TF: map -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = pose.pose.orientation.z
        t.transform.rotation.w = pose.pose.orientation.w
        self.tf_pub.sendTransform(t)

        # 4) Perception: publish local_cones at ~10 Hz
        self._cones_accum += self.dt
        cones_period = 1.0 / max(self.cones_rate_hz, 1e-6)
        if self._cones_accum + 1e-9 >= cones_period:
            self._cones_accum = 0.0
            self._publish_local_cones(now)

    def _publish_local_cones(self, now):
        msg = PoseArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'  # per your spec

        # Transform each world cone into base_link at current pose
        cos_y = math.cos(self.psi); sin_y = math.sin(self.psi)
        count = 0
        for (cx, cy) in self.world_cones:
            dx, dy = cx - self.x, cy - self.y
            # map -> base_link: rotate by -psi
            lx =  dx *  cos_y + dy * sin_y
            ly = -dx *  sin_y + dy * cos_y
            if lx < 0.0 or lx > self.range_x_max or abs(ly) > self.range_y_abs_max:
                continue
            p = Pose()
            p.position.x = float(lx)
            p.position.y = float(ly)
            # TEMP: isTall shim in z (0.0 short, 1.0 tall). YAML has no tall info, default short.
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)
            count += 1
            if count >= self.max_cones:
                break
        self.cones_pub.publish(msg)

def main():
    rclpy.init()
    node = SimLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
