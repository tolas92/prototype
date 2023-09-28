import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('Imu_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        self.subscription=self.create_subscription(LaserScan,'/scan',self.laser_callback,10)

    def laser_callback(self,msg):
        scan_time=msg.
        # Create a TransformStamped message for the TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'base_link'  # Assuming the pose is in the "world" frame
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

    def odom_callback(self, msg):
        qz = msg.orientation.z
        qw = msg.orientation.w

    
def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
