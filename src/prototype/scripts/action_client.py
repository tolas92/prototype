import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node

from prototype.action import GoTo


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self,GoTo, '/table_nav')

    def send_goal(self, msg):
        goal_msg = GoTo.Goal()
        goal_msg.x = msg[0]
        goal_msg.y=msg[1]

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
    
    def feedback_callback(self,feedback_msg):
        feedback=feedback_msg.feedback
        self.get_logger().info(f'Feedback: x={feedback.distance_left}')






def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal([10.0,20.0])

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()