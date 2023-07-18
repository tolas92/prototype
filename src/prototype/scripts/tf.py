import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/camera_pose',
            self.pose_callback,
            10
        )
        self.subscription

    def pose_callback(self, msg):
        # Process the pose data from the /camera_pose topic
        # Assuming the pose data contains position (x, y, z) and orientation (quaternion)

        # Assuming the data is available in the variables: x, y, z, qx, qy, qz, qw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Create a TransformStamped message for the TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'plane'  # Assuming the pose is in the "world" frame
        tf_msg.child_frame_id = 'imu_link'  # Assuming the pose is in the "imu_link" frame

        # Set the translation
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z

        # Set the rotation (quaternion)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        # Publish the TF
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
