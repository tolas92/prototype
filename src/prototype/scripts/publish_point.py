import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class DetectBall(Node):

    def __init__(self):
        super().__init__('detect_ball')
        self.ball_pub = self.create_publisher(Imu, "/imu/data_raw", 10)

        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every 1 second

    def timer_callback(self):
        # For testing purposes, just publish fixed ball coordinates
        msg=Imu()
        msg.angular_velocity.x=0.0
        msg.angular_velocity.y=0.0
        msg.angular_velocity.z=1.0
        msg.orientation.x=1.232
        msg.orientation.y=2.2
        msg.orientation.z=1.2
        msg.orientation.w=1.0
        msg.linear_acceleration.x=2.0
        self.ball_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    detect_ball = DetectBall()
    rclpy.spin(detect_ball)

    detect_ball.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
