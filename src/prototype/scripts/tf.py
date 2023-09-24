import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        # Process the pose data from the /camera_pose topic
        # Assuming the pose data contains position (x, y, z) and orientation (quaternion)
        # Assuming the data is available in the variables: x, y, z, qx, qy, qz, qw
        x = msg.angular_velocity.x
        y = msg.angular_velocity.y
        z = msg.angular_velocity.z
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Create a TransformStamped message for the TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'chassis'  # Assuming the pose is in the "world" frame
        tf_msg.child_frame_id = 'imu_link'  # Assuming the pose is in the "imu_link" frame

        # Set the translation
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0

        # Set the rotation (quaternion)
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
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
