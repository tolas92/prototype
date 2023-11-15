import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
class GoalPosePublisher(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscriber=self.create_subscription(PoseStamped,'/person_pose',self.publish_goal_pose,10)
    def publish_goal_pose(self,msg):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = msg.pose.position.x
        pose_msg.pose.position.y = msg.pose.position.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.publisher.publish(pose_msg)
        self.get_logger().info("Published goal pose.")
       # time.sleep(1)

    

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

