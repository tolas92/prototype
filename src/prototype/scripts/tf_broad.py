import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import     Imu
import tf2_ros

class TFBroadcasterIMU(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_imu')
        self.tf_broadcaster=tf2_ros.TransformBroadcaster(self)
        self.subscription=self.create_subscription(Imu,'/imu/data_raw',self.handle_imu_pose,10)
    
    def handle_imu_pose(self,msg):
        t=TransformStamped()
        t.header.stamp =self.get_clock().now().to_msg()
        t.header.frame_id='plane'
        t.child_frame_id='imu_link'
        t.transform.translation.x=0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x=msg.orientation.x
        t.transform.rotation.y=msg.orientation.y
        t.transform.rotation.z=msg.orientation.z
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
  rclpy.init(args=args)
  tf_broadcaster_imu=TFBroadcasterIMU()
  rclpy.spin(tf_broadcaster_imu)
  tf_broadcaster_imu.destroy_node()
  rclpy.shutdown()

if __name__=='__main__':
    main()

        
        
        