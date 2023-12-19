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

import rclpy
from rclpy.node import Node

from ue_msgs.msg import EntityState
from ue_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Quaternion

import json
import numpy as np
import quaternion
from enum import Enum

def array_to_size_param(size):
    return {'X': size[0], 'Y': size[1], 'Z': size[2]}

class ModelNames(Enum):
    PHYSICS_CUBE = 'BP_PhysicsCube'
    NON_PHYSICS_CUBE = 'BP_NonPhysicsCube'
    CONVEYOR = 'BP_Conveyor'
    SPLINE_CONVEYOR = 'BP_Spline_Conveyor'
    ELEVATOR = 'BP_Elevator2S'
    VERTICAL_CONVEYOR = 'BP_VerticalConveyor'

class ExternalDeviceClient(Node):
    payload_id = 0
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.future = None
        self.declare_parameter('debug', False)
        self.declare_parameter('enable_widget', True)
        self.declare_parameter('disable_physics', False)
        self.declare_parameter('mode', 0)
        self.declare_parameter('size', [1.0, 1.0, 1.0])
        self.declare_parameter('spawn_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('payload_spawn_pose', [0.0, 0.0, 3.0, 0.0, 0.0, 0.0])
    
    #     timer_period = 1  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    
    # def timer_callback(self):
    #     self.get_logger().info('Publishing: "%s"' % msg.data)

    def parse_size_param(self):
        return array_to_size_param(self.get_parameter('size').value)

    def spawn_model(self, pose, model_name, namespace, name, tag, json_parameters):
        q = quaternion.from_euler_angles(pose[3], pose[4], pose[5])
        initial_pose = Pose()
        initial_pose.position.x = pose[0]
        initial_pose.position.y = pose[1]
        initial_pose.position.z = pose[2] 
        initial_pose.orientation.x = q.x
        initial_pose.orientation.y = q.y
        initial_pose.orientation.z = q.z
        initial_pose.orientation.w = q.w

        req = SpawnEntity.Request()
        req.xml = model_name
        req.robot_namespace = namespace
        req.state = EntityState()
        req.state.name = name
        req.state.pose = initial_pose
        req.tags = [tag]
        req.json_parameters = json.dumps(json_parameters)

        srv_client = self.create_client(SpawnEntity, '/SpawnEntity')
        self.future = srv_client.call_async(req)

        def cb(future):
            res = future.result()
            if res:
                self.get_logger().info('Successfully spawned {}'.format(name))
            else:
                self.get_logger().error('Failed to spawn {}'.format(name))
        
        self.future.add_done_callback(cb)

    def spawn_payload(self, payload_model):
        self.spawn_model(self.get_parameter('payload_spawn_pose').value, payload_model, '', 'payload'+str(ExternalDeviceClient.payload_id), 'Payload', {})
        ExternalDeviceClient.payload_id += 1

    def delete_model(self, name):
        req = DeleteEntity.Request()
        req.name = name
        srv_client = self.create_client(DeleteEntity, '/DeleteEntity')
        self.future = srv_client.call_async(req)

        def cb(future):
            res = future.result()
            if res:
                self.get_logger().info('Successfully deleted {}'.format(name))
            else:
                self.get_logger().error('Failed to delete {}'.format(name))
        
        self.future.add_done_callback(cb)
    