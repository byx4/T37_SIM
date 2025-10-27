import os, yaml, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class ConeMapPublisher(Node):
    def __init__(self):
        super().__init__("cone_map_pub")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("viz_frame", "map")
        self.declare_parameter("point_size", 0.35)

        path = self.get_parameter("map_yaml").get_parameter_value().string_value
        if not path or not os.path.exists(path):
            raise RuntimeError(f"map_yaml not found: {path}")

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        yaml_frame = str(data.get("frame_id", "map"))
        cones = data.get("cones", []) or []

        pts = []
        for c in cones:
            try:
                pts.append((float(c["x"]), float(c["y"])))
            except Exception:
                pass

        self.get_logger().info(f"Loaded {len(pts)} cones from {path}")
        if not pts:
            self.get_logger().warn("No cones parsed; will publish an empty array")

        frame = self.get_parameter("viz_frame").get_parameter_value().string_value or yaml_frame
        size = float(self.get_parameter("point_size").value)

        m = Marker()
        m.header.frame_id = frame
        m.ns = "global_cones"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = size
        m.scale.y = size
        m.color.r, m.color.g, m.color.b, m.color.a = (0.84, 0.13, 0.13, 1.0)
        m.points = [Point(x=x, y=y, z=0.0) for (x, y) in pts]

        self.msg = MarkerArray(markers=[m])

        # Transient-local = latched, so late subscribers (Foxglove) still get it
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, "/global_cones", qos)

        # Publish immediately, then keep re-stamping once per second
        self._publish_now()
        self.timer = self.create_timer(1.0, self._publish_now)

    def _publish_now(self):
        now = self.get_clock().now().to_msg()
        for m in self.msg.markers:
            m.header.stamp = now
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = ConeMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
