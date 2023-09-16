import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64MultiArray
from functools import partial
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('Velocity_message')
        self.publisher_=self.create_publisher(Float64MultiArray,'velocity_controller/commands',10)
        timer_period=1
        self.timer=self.create_timer(timer_period,self.timer_callback)
        
       
    def timer_callback(self):
        msg=Float64MultiArray()
        msg.data.append(0.0)
        #msg._layout.dim.append(1)
        #msg._layout.data_offset=1
        self.publisher_.publish(msg)
        #time.sleep(1)
        #msg.data[0]=1.0
        #self.publisher_.publish(msg)
        #self.get_logger().debug('Publishing: "%s"' % msg)
        self.get_logger().info('Publishing: %s' % msg)
    
        

    
def main(args=None):
    rclpy.init()
    simple_publisher=SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()


