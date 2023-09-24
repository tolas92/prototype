import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from builtin_interfaces.msg import Time

class GoalSub(Node):
    def __init__(self):
        super().__init__('goal_sub')
        self.subscription = self.create_subscription(String,"/string_pub",self.callback,10)
     #   self.publisher=self.create_publisher(String,"/string_",10)

    def callback(self,msg):
        print("Received gesture:",msg.data)
      #  self.publisher.publish(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = GoalSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
