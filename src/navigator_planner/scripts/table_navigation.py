#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer ,GoalResponse,CancelResponse
from prototype.action import SendCoordinates
from prototype.action._send_coordinates import SendCoordinates_Goal
from action_msgs.msg import GoalStatus


class AiCommand(Node):

    def __init__(self):
        super().__init__('AiCommand')
        self.command_ = self.create_subscription(String, '/hand_gesture',self.callback, 10)
        self.action_server_=ActionServer(self,SendCoordinates,'/home',self.callback)
        self.navigator = BasicNavigator()

    def callback(self,goal_handle): 
        feedback_msg=SendCoordinates.Feedback()
        """
        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = 2.15
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

        """
        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # self.navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # If desired, you can change or load the map as well
        # self.navigator.changeMap('/path/to/map.yaml')

        # You may use the self.navigator to clear or obtain costmaps
        # self.navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = self.navigator.getGlobalCostmap()
        # local_costmap = self.navigator.getLocalCostmap()
            # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.z=0.0
        goal_pose.pose.orientation.w = 1.0

            # sanity check a valid path exists
            # path = self.navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)

            # sanity check a valid path exists
            # path = self.navigator.getPath(initial_pose, goal_pose)

        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            
            feedback = self.navigator.getFeedback()
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                   Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
            """   
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                   # self.navigator.goToPose(goal_pose)
            """

        # Do something depending on the return code
        result = self.navigator.getResult()
        result_=SendCoordinates.Result()
        if result == TaskResult.SUCCEEDED:
            goal_handle.succeed()
            result_.success=True
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            goal_handle.canceled()
            result_.success=False
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            goal_handle.abort()
            result_.success=False
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
        return result_

        #self.navigator.lifecycleShutdown()

        #exit(0)

def main(args=None):
    rclpy.init(args=args)
    aicommand = AiCommand()
    rclpy.spin(aicommand)
    # If we press control + C, the node will stop.
    aicommand.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()