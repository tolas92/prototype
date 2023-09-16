import sys
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from functools import partial

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('Twist_message')
        
       
    def timer_callback(self,speed):
        msg=Twist()
        msg.linear.x=speed
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
    
    def call_publisher(self,speed):
        self.publisher_=self.create_publisher(Twist,'turtle1/cmd_vel',10)
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


