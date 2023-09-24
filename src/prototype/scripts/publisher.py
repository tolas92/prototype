import sys
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from functools import partial

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('Twist_message')
        
       
    def timer_callback(self,speed):
        msg=Imu()
        msg.angular_velocity.x=0.0
        msg.angular_velocity.y=0.0
        msg.angular_velocity.z=1.0
        msg.orientation.x=1.232
        msg.orientation.y=2.2
        msg.orientation.z=1.2
        msg.orientation.w=1.0
        msg.linear_acceleration.x=2.0
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg)
    
    def call_publisher(self,speed):
        self.publisher_=self.create_publisher(Imu,'/imu/data_raw',10)
        time_period=0.5
        self.timer_=self.create_timer(time_period,partial(self.timer_callback,speed))
        

    
def main(args=None):
    rclpy.init()
    speed=float(sys.argv[1])
    simple_publisher=SimplePublisher()
    simple_publisher.call_publisher(speed)
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()


