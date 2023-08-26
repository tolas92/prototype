import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DetectBall(Node):

    def __init__(self):
        super().__init__('detect_ball')

        self.get_logger().info('Looking for the ball...')
        self.ball_pub = self.create_publisher(PoseStamped, "/detected_ball", 1)

        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every 1 second

    def timer_callback(self):
        # For testing purposes, just publish fixed ball coordinates
        point_out = PoseStamped()
        point_out.pose.position.x = 1.0
        point_out.pose.position.y = 2.0
        point_out.pose.position.z = 3.0
        self.ball_pub.publish(point_out)

def main(args=None):
    rclpy.init(args=args)

    detect_ball = DetectBall()
    rclpy.spin(detect_ball)

    detect_ball.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
