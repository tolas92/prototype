import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from builtin_interfaces.msg import Time

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'object_link'

        msg.pose.position = Point(x=.08, y=0.0, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.publisher.publish(msg)
        self.get_logger().info("Published goal pose")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
