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
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32, Float32, Int32MultiArray, String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Pose, Point

from .common import ExternalDeviceClient, ModelNames, AIMoveMode, array_to_vector_param, array_to_pose_param


##########################################################################################
# Character Control client example
#     1. spawn character
#     2. Keep sending random goal if it is idle.
##########################################################################################

from enum import Enum
class CharacterMoveStatus(Enum):
    IDLE   = 0
    MOVING = 1

class AITaskType(Enum):
    NONE      = 'None'
    MOVE      = 'Move'
    PICK      = 'Pick'
    DROP      = 'Drop'

# same as defined in RRAIRobotROSController.h
class AINavigationStatus(Enum):
    IDLE            = 0
    AI_MOVING       = 1
    LINEAR_MOVING   = 2
    ROTATING        = 3


class AIControlledActorClient(ExternalDeviceClient):
    def __init__(self, name, spawn=True, **kwargs):        
        super().__init__(name, spawn, **kwargs)
        self.nav_status = AINavigationStatus.IDLE.value
        self.task_status = AITaskType.NONE.value

    def ros_api_settings(self):
        super().ros_api_settings()

        #############
        # parameters
        #############
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('origin', 'MapOrigin')
        self.declare_parameter('goal_sequence', "[]")
        self.declare_parameter('random_move_bounding_box', [0.0, 0.0, 0.0])

        # example usage of client
        self.declare_parameter('random_move_by_ros', False) 
        self.random_move_by_ros = self.get_parameter('random_move_by_ros').value
        
        #############
        # pub/sub
        #############
        # pub
        self.mode_publisher_ = self.create_publisher(Int32, 'set_mode', 10)
        # navigation
        self.pose_goal_publisher_ = self.create_publisher(PoseStamped, 'pose_goal', 10)
        self.actor_goal_publisher_ = self.create_publisher(String, 'actor_goal', 10)
        # pick drop
        self.pick_goal_publisher_ = self.create_publisher(PointStamped, 'pick_goal', 10)
        self.pick_actor_goal_publisher_ = self.create_publisher(String, 'pick_actor_goal', 10)
        self.drop_goal_publisher_ = self.create_publisher(PoseStamped, 'drop_goal', 10)
        self.drop_actor_goal_publisher_ = self.create_publisher(String, 'drop_actor_goal', 10)
        self.set_approach_location_publisher_ = self.create_publisher(PointStamped, 'set_approach_location', 10)
        self.set_approach_location_actor_publisher_ = self.create_publisher(String, 'set_approach_location_actor', 10)
        self.set_use_default_approach_publisher_ = self.create_publisher(Bool, 'set_use_default_approach', 10)
                
        # sub 
        self.nav_status_subscription = self.create_subscription(
            Int32,
            'nav_status',
            self.nav_status_cb,
            10)
        self.nav_status_subscription  # prevent unused variable warning
        self.task_status_subscription = self.create_subscription(
            Int32,
            'task_status',
            self.task_status_cb,
            10)
        self.task_status_subscription  # prevent unused variable warning


        self.mode = self.get_parameter('mode').value
        # self.payload_status = False 
        self.speed = self.get_parameter('speed').value

        self.json_parameters = {
            'speed': self.speed,
            'debug': self.get_parameter('debug').value,
            'mode': self.mode,
            'enable_widget': self.get_parameter('enable_widget').value,
            'origin': self.parse_origin(),
            'goal_sequence': self.parse_point_sequence(self.get_parameter('goal_sequence').value),
            'random_move_bounding_box': array_to_vector_param(self.get_parameter('random_move_bounding_box').value),
        }        

    def parse_origin(self):
        origin_str = self.get_parameter('origin').value
        try: # if it given as coordinate
            origin_list = eval(origin_str)
            if len(origin_list) == 6:
                return array_to_pose_param(origin_list)
        except: # if it given as actor name
            return origin_str

    def nav_status_cb(self, msg):
        self.nav_status = msg.data
        print('nav_status_cb in AIControlledActorClient', msg.data)
        if msg.data == CharacterMoveStatus.IDLE.value and self.random_move_by_ros: # reached goal
            if self.mode == AIMoveMode.MANUAL.value: # manual mode
                goal = PoseStamped()
                goal.header.frame_id: 'MapOrigin' # map origin actor name. If the actor does not exist, origin will become world origin
                goal.pose.position.x = random.uniform(-10, 10)
                goal.pose.position.y = random.uniform(-10, 10) 
                goal.pose.position.z = 0.0
                self.manual_goal_publisher_.publish(goal)
                self.get_logger().info('send manual goal to {}'.format(goal))

    def task_status_cb(self, msg):
        print('task_status_cb in AIControlledActorClient', msg.data)
        self.task_status = msg.data

def main(args=None):
    rclpy.init(args=args)

    ai_controlled_actor_client = AIControlledActorClient('ai_controlled_actor_client')

    rclpy.spin(ai_controlled_actor_client)

    ai_controlled_actor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
