# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import time
import random
import quaternion

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState

from example_interfaces.msg import Int32, Float32, Int32MultiArray
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose

from .common import ExternalDeviceClient, ModelNames, array_to_vector_param, array_to_pose_param


##########################################################################################
# Character Control client example
#     1. spawn character
#     2. Keep sending random goal if it is idle.
##########################################################################################

from enum import Enum

class TB3Client(ExternalDeviceClient):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # spawn self and payload
        namespace = self.get_namespace()
        entity_name = self.get_parameter('robot_name').value
        if entity_name == '':
            entity_name = namespace[1:len(namespace)] # remove /
        if self.spawn:
            self.spawn_self(self.get_parameter('spawn_pose').value, entity_name, namespace, '', self.json_parameters)

    def ros_api_settings(self):
        super().ros_api_settings()

        self.declare_parameter('robot_name', 'tb3')
        self.declare_parameter('map_origin', 'map')
        self.declare_parameter('goal_sequence', '[]')

        self.initialized = False

        pose = self.get_parameter('spawn_pose').value
        q = quaternion.from_euler_angles(pose[3], pose[4], pose[5])
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = self.get_parameter('map_origin').value
        self.initial_pose.pose.pose.position.x = pose[0]
        self.initial_pose.pose.pose.position.y = pose[1]
        self.initial_pose.pose.pose.position.z = pose[2] 
        self.initial_pose.pose.pose.orientation.x = q.x
        self.initial_pose.pose.pose.orientation.y = q.y
        self.initial_pose.pose.pose.orientation.z = q.z
        self.initial_pose.pose.pose.orientation.w = q.w
        self.initial_pose.pose.covariance[0] = self.initial_pose.pose.covariance[7] = 0.25
        self.initial_pose.pose.covariance[35] = 0.06853891909122467
        
        self.goals = self.load_pose_sequence(self.get_parameter('goal_sequence').value)
        self.current_goal_index = 0

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)


        #action client to send nav goal
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._pose_subscriber = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose',
                                                       self.amcl_pose_cb,
                                                       amcl_pose_qos)
        self._initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        
        self.json_parameters = {
            'debug': self.get_parameter('debug').value,
            'enable_widget': self.get_parameter('enable_widget').value,
        }  
    
    def amcl_pose_cb(self, msg):
        self.initialized = True

    def send_goal(self):
        print('sendgoal', self.initialized)
        if not self.initialized:
            self.initial_pose.header.stamp = self.get_clock().now().to_msg()
            self._initial_pose_publisher.publish(self.initial_pose)
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.get_parameter('map_origin').value
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        current_goal = self.goals[self.current_goal_index]
        q = quaternion.from_euler_angles(current_goal[3], current_goal[4], current_goal[5])
        goal_msg.pose.pose.position.x = current_goal[0]
        goal_msg.pose.pose.position.y =  current_goal[1]
        goal_msg.pose.pose.position.z =  current_goal[2]
        goal_msg.pose.pose.orientation.x = q.x
        goal_msg.pose.pose.orientation.y = q.y
        goal_msg.pose.pose.orientation.z = q.z
        goal_msg.pose.pose.orientation.w = q.w
        
        self._action_client.wait_for_server()

        self.get_logger().info(('Navigating to goal: '))

        send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_cb)

        res = rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=1.0)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return False

        result_future = self.goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(('Navigation completed: ' + str(result_future.result().status)))
        
        if len(self.goals) <= self.current_goal_index:
            self.current_goal = 0
        else:
            self.current_goal_index += 1

        return True

    def parse_goal_sequence(self):
        points_str = self.get_parameter('goal_sequence').value
        points = eval(points_str)
        output = []
        for p_str in points:
            try: # if it given as coordinate
                p = eval(p_str)
                if len(p) == 3:
                    output.append({'position': array_to_vector_param(p)})
                else:
                    self.get_logger().info('spline points length should be 3')
            except:  # if it given as actor name
                output.append({'name': p_str})
        
        return output
    
    def feedback_cb(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    tb3_client = TB3Client('tb3_client')

    while rclpy.ok():
        res = tb3_client.send_goal()
        print('test0', res)
        rclpy.spin_once(tb3_client)
        print('test1', res)
        time.sleep(1)

    tb3_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
