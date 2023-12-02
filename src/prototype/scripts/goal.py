import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPosePublisher(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer=self.create_timer(100,self.publish_goal_pose)
    def publish_goal_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp.sec = 0
        pose_msg.header.stamp.nanosec = 0
        pose_msg.pose.position.x = 0.02
        pose_msg.pose.position.y = -0.06
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.publisher.publish(pose_msg)
        self.get_logger().info("Published goal pose.")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    node.publish_goal_pose()  # Publish goal pose once
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

