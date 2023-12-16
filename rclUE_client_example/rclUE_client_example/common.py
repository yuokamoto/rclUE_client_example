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

class ExternalDeviceClient(Node):
    payload_id = 0
    def __init__(self, name):
        super().__init__(name)
        
    def spawn_model(self, pose, model_name, namespace, name, tag, json_parameters):
        q = quaternion.from_euler_angles(pose['roll'], pose['pitch'], pose['yaw'])
        initial_pose = Pose()
        initial_pose.position.x = pose['x']
        initial_pose.position.y = pose['y']
        initial_pose.position.z = pose['z'] 
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
        rclpy.spin_until_future_complete(self, self.future)
        res = self.future.result()

        if res:
            self.get_logger().info('Successfully spawned {}'.format(name))
        else:
            self.get_logger().error('Failed to spawn {}'.format(name))

    def spawn_payload(self, pose, payload_model):
        print(ExternalDeviceClient.payload_id)
        self.spawn_model(pose, payload_model, '', 'payload'+str(ExternalDeviceClient.payload_id), 'Payload', {})
        ExternalDeviceClient.payload_id += 1

    def delete_model(self, name):
        req = DeleteEntity.Request()
        req.name = name
        srv_client = self.create_client(DeleteEntity, '/DeleteEntity')
        self.future = srv_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        res = self.future.result()

        if res:
            self.get_logger().info('Successfully deleted {}'.format(name))
        else:
            self.get_logger().error('Failed to delete {}'.format(name))
    
from enum import Enum
class ModelNames(Enum):
    CUBE = 'CUBE'
    CONVEYOR = 'BP_Conveyor'
    SPLINE_CONVEYOR = 'BP_Spline_Conveyor'
    ELEVATOR = 'BP_Elevator2S'
    VERTICAL_CONVEYOR = 'BP_VerticalConveyor'
