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

def array_to_vector_param(in_array):
    return {'x': in_array[0], 'y': in_array[1], 'z': in_array[2]}

def array_to_rotation_param(in_array):
    return {'roll': in_array[0], 'pitch': in_array[1], 'yaw': in_array[2]}

def array_to_pose_param(in_array):
    return {
        'position': array_to_vector_param(in_array[:3]), 
        'rotation': array_to_rotation_param(in_array[3:])
    }

# UE assets name. This is specified in DefaultRapyutaSimSettings.ini
class ModelNames(Enum):
    # payload
    PHYSICS_CUBE = 'BP_PhysicsCube'
    NON_PHYSICS_CUBE = 'BP_NonPhysicsCube'
    # conveyor
    CONVEYOR = 'BP_Conveyor'
    BELT_CONVEYOR = 'BP_BeltConveyor'
    ROLLER_CONVEYOR = 'BP_RollerConveyor'
    # spline conveyor
    SPLINE_CONVEYOR = 'BP_SplineConveyor'
    BELT_SPLINE_CONVEYOR = 'BP_BeltSplineConveyor'
    ROLLER_SPLINE_CONVEYOR = 'BP_RollerSplineConveyor'
    # vertical movement
    ELEVATOR = 'BP_Elevator2S'
    VERTICAL_CONVEYOR = 'BP_VerticalConveyor'
    
    @classmethod
    def value_of(cls, value):
        for k, v in cls.__members__.items():
            if k == value:
                return v
        else:
            raise ValueError(f"'{cls.__name__}' enum not found for '{value}'")

class AIMoveMode(Enum):
    BEGIN = 0

    MANUAL          = 1
    SEQUENCE        = 2
    RANDOM_SEQUENCE = 3
    RANDOM_AREA     = 4

    END             = 100

class ExternalDeviceClient(Node):
    def __init__(self, name, spawn=True, model_name='BP_Conveyor', reference_frame='', initial_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], **kwargs):
        super().__init__(name, **kwargs)
        self.future = None
        self.model_name = model_name
        self.payload_id = 0
        self.spawn = spawn
        self.reference_frame = reference_frame
        self.initial_pose = initial_pose

        # parameters, pub/sub/service
        self.ros_api_settings()

        # spawn self
        namespace = self.get_namespace()
        self.entity_name = namespace[1:len(namespace)] # remove / 
        if self.spawn:
            self.spawn_self(self.get_parameter('spawn_pose').value, self.entity_name, self.entity_name, '', self.json_parameters)

    def ros_api_settings(self):
        # common ROS parameters
        self.declare_parameter('spawn', self.spawn)
        self.declare_parameter('reference_frame', self.reference_frame)
        self.declare_parameter('debug', False)
        self.declare_parameter('enable_widget', True)
        self.declare_parameter('disable_physics', False)
        self.declare_parameter('mode', 0)
        self.declare_parameter('size', [1.0, 1.0, 1.0])
        self.declare_parameter('spawn_pose', self.initial_pose)
        self.declare_parameter('payload_spawn_pose', [0.0, 0.0, 3.0, 0.0, 0.0, 0.0])
        self.declare_parameter('model_name', self.model_name)

        self.spawn = self.get_parameter('spawn').value
        self.reference_frame = self.get_parameter('reference_frame').value
        if self.spawn:
            # get model name
            self.model_name = self.get_parameter('model_name').value
            if self.model_name == '':
                self.get_logger().error('You must provide valid model name as a ROS parameter')
                self.destroy_node() 

            # service clients
            self.spawn_srv_client = self.create_client(SpawnEntity, '/SpawnEntity')
            while not self.spawn_srv_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('SpawnEntity not available')
                self.destroy_node() 
            
            self.delete_srv_client = self.create_client(DeleteEntity, '/DeleteEntity')
            while not self.delete_srv_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('DeleteEntity not available')
                self.destroy_node() 

    
    def parse_size_param(self):
        return array_to_vector_param(self.get_parameter('size').value)

    def parse_point_sequence(self, points_str):
        points = eval(points_str)
        output = []
        for p_str in points:
            try: # if it given as coordinate
                p = eval(p_str)
                if len(p) == 6:
                    output.append({
                        'pose': {
                           'position': array_to_vector_param(p[0:3]),
                           'orientation': array_to_rotation_param(p[3:6])
                        }
                    })
                else:
                    self.get_logger().info('points length should be 6')
            except:  # if it given as actor name
                output.append({'name': p_str})
        
        return output
    
    def load_pose_sequence(self, points_str):
        points = eval(points_str)
        output = []
        for p_str in points:
            try: # if it given as coordinate
                p = eval(p_str)
                if len(p) == 6:
                    output.append(p)
                else:
                    self.get_logger().info('points length should be 3')
            except:
                continue
        
        return output

    def spawn_model(self, pose, model_name, name, namespace, tag, json_parameters):
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
        req.state.reference_frame = self.reference_frame 
        req.tags = [tag]
        req.json_parameters = json.dumps(json_parameters)

        self.get_logger().info('Send spawn request of {}, request: \n {}'.format(name, req))
        self.future = self.spawn_srv_client.call_async(req)

        def cb(future):
            res = future.result()
            if res:
                self.get_logger().info('Successfully spawned {}'.format(name))
            else:
                self.get_logger().error('Failed to spawn {}'.format(name))
        
        self.future.add_done_callback(cb)
    
    def spawn_self(self, pose, name, namespace, tag, json_parameters):
        self.spawn_model(pose, self.model_name, name, namespace, tag, json_parameters)

    def spawn_payload(self, payload_model):
        namespace = self.get_namespace()
        entity_name = namespace[1:len(namespace)]
        self.spawn_model(self.get_parameter('payload_spawn_pose').value, payload_model, namespace + '_payload'+str(self.payload_id), '', 'Payload', {})
        self.payload_id += 1

    def delete_model(self, name):
        req = DeleteEntity.Request()
        req.name = name
        self.future = self.delete_srv_client.call_async(req)

        def cb(future):
            res = future.result()
            if res:
                self.get_logger().info('Successfully deleted {}'.format(name))
            else:
                self.get_logger().error('Failed to delete {}'.format(name))
        
        self.future.add_done_callback(cb)
    