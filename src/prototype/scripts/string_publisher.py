import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from builtin_interfaces.msg import Time

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('string_publisher')
        self.publisher = self.create_publisher(String, '/hand_gesture', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data="go"
        self.publisher.publish(msg)
        self.get_logger().info("Published goal pose")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
