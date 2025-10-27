import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        self.declare_parameter('pose_topic', '/vehicle_pose')
        self.declare_parameter('base_frame', 'base_link')
        self.br = TransformBroadcaster(self)
        topic = self.get_parameter('pose_topic').value
        self.base = self.get_parameter('base_frame').value
        self.create_subscription(PoseStamped, topic, self.cb, 10)

    def cb(self, msg: PoseStamped):
        t = TransformStamped()
        t.header = msg.header           # expects frame_id = 'map'
        t.child_frame_id = self.base
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(PoseToTF())
    rclpy.shutdown()
