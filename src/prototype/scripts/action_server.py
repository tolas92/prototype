import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from prototype.action import SendCoordinates


class CoActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            SendCoordinates,
            '/home',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        goal=goal_handle.request
        x=goal.x
        y=goal.y
        feedback_msg=SendCoordinates.Feedback()
        for i in range(1, 11):  # Loop for 10 seconds
            feedback_msg.current_x = float(i)  # Update x-coordinate
            feedback_msg.current_y = float(i) 
            self.get_logger().info(f'Feedback: x={x}, y={y}')
            goal_handle.publish_feedback(feedback_msg)         
            time.sleep(1)

        goal_handle.succeed()
        result = SendCoordinates.Result()
        time.sleep(20)
        result.success=True
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = CoActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()