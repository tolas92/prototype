import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseModifierNode(Node):

    def __init__(self):
        super().__init__('pose_modifier_node')

        # Create a subscriber to the input topic
        self.subscription = self.create_subscription(
            PoseStamped,
            'laser_pose',
            self.pose_callback,
            10
        )

        # Create a publisher for the modified pose
        self.publisher = self.create_publisher(PoseStamped, 'laser2_pose', 10)

    def pose_callback(self, msg):
        # Modify the incoming pose
        modified_pose = PoseStamped()
        modified_pose.header = msg.header
        modified_pose.pose.position.x = msg.pose.position.x
        modified_pose.pose.position.y = 0.0  # Set Y component to zero
        modified_pose.pose.position.z = 0.0  # Set Z component to zero
        modified_pose.pose.orientation = msg.pose.orientation

        # Publish the modified pose
        self.publisher.publish(modified_pose)
        self.get_logger().info('Modified pose published')

def main(args=None):
    rclpy.init(args=args)
    node = PoseModifierNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
