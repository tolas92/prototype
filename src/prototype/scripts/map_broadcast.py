import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion,Point
from tf2_ros import TransformBroadcaster
import math

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_map')
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.pose_subscriber=self.create_subscription(Point,"/detected_ball",self.callback,1)
        self.timer = self.create_timer(1.0, self.callback)
        self.map_frame = 'map'
        self.robot_frame = 'base_link'

    def callback(self):
        
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = self.map_frame
        transform_stamped.child_frame_id = self.robot_frame

        # Set the translation and rotation from the constant pose
        transform_stamped.transform.translation.x = 1.0#msg.x
        transform_stamped.transform.translation.y = 0.0#0msg.y
        transform_stamped.transform.translation.z = 0.0#msg.z

        # Set the rotation (quaternion)
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0

        # Publish the transformation
        self.tf_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
