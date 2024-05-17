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
from example_interfaces.msg import Int32, Float32, Int32MultiArray, Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Pose, Point

from .common import ExternalDeviceClient, ModelNames, AIMoveMode, array_to_vector_param, array_to_pose_param

from .ai_actor_client import AIControlledActorClient, AITaskType, AINavigationStatus

##########################################################################################
# Character Control client example
#     1. spawn character
#     2. Keep sending random goal if it is idle.
##########################################################################################


class WarehouseClient(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.task_list = {}
        self.agent_list = []
        # todo handle non agent list tasks

        self.ros_api_settings()
        self.parse_csv()

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.process_order)

    #     self.status_subscription = self.create_subscription(
    #         Int32,
    #         'nav_status',
    #         self.status_cb,
    #         10)
    #     self.status_subscription  # prevent unused variable warning
    
    # def status_cb(self, msg):
    #     print('status_cb in WarehouseClient', msg.data)

    def ros_api_settings(self):
        pass

    def parse_csv(self):
        self.agent_list.append(AIControlledActorClient('AIControlledActorClient', spawn=False, namespace='BP_ROSCharacter14'))
        self.task_list['BP_ROSCharacter14'] = [
            {
                'task_id': 'task_1',
                'task_type': 'Pick', 
                # 'goal_location': [],
                # 'goal_orientation': []
                'goal_actor': 'BP_Cart',
                # 'approach_location':[]
                # 'approach_actor':
                'use_default_apporach_location': True,
                'completed': False
            },
            {
                'task_id': 'task_2',
                'task_type': 'Drop', 
                'goal_actor': 'DropPoint',
                'approach_actor': 'ApproachPoint2'
                'completed': False
            }
            
        ]
        # load csv
        # parse csv to dict
        # create task list for agents
        # create pub/sub
    
    def check_task_dependency(self):
        # get list o task
        # get list of tasks with dependency
        # return true if dependent task is completed
        pass

    def process_order(self):
        print('process_order')
        for agent in self.agent_list:
            rclpy.spin_once(agent, timeout_sec=0.001)
            if agent.task_status == AITaskType.NONE.value and agent.nav_status == AINavigationStatus.IDLE.value and len(self.task_list[agent.entity_name]) > 0:
                print('[{}] send next task'.format(agent.entity_name))
                #todo check task dependency 
                task = self.task_list[agent.entity_name].pop(0)

                if 'task_type' not in task:
                    print('task_type not found in task {}'.format(task['task_id'] if 'task_id' in task else 'None'))
                    continue

                # parse param
                use_default_apporach_location = task['use_default_apporach_location'] if 'use_default_apporach_location' in task else False
                goal_loction = task['goal_location'] if 'goal_location' in task else None
                goal_orientation = task['goal_orientation'] if 'goal_orientation' in task else None
                goal_actor = task['goal_actor'] if 'goal_actor' in task else None
                approach_location = task['approach_location'] if 'approach_location' in task else None
                approach_actor = task['approach_actor'] if 'approach_actor' in task else None
            
                # parse goal location
                goal_actor_msg = goal_location_msg = goal_orientation_msg = point_stamped_msg = pose_stamped_msg = None
                if goal_actor is not None and goal_actor != '':
                    goal_actor_msg = String()
                    goal_actor_msg.data = goal_actor

                elif goal_loction is not None and len(goal_loction) >= 3:
                    goal_location_msg = Point()
                    goal_location_msg.x = goal_location[0]
                    goal_location_msg.y = goal_location[1]
                    goal_location_msg.z = goal_location[2]

                    point_stamped_msg = PointStamped()
                    point_stamped_msg.header.frame_id = self.reference_frame
                    point_stamped_msg.point = goal_location_msg
                
                elif goal_orientation is not None and len(goal_orientation) >= 3:
                    goal_orientation_msg = Orientation()
                    q = quaternion.from_euler_angles(goal_orientation[0], goal_orientation[1], goal_orientation[2])
                    goal_orientation_msg.x = q.x
                    goal_orientation_msg.y = q.y
                    goal_orientation_msg.z = q.z
                    goal_orientation_msg.w = q.w
                
                if goal_location_msg is not None and goal_orientation_msg is not None:
                    pose_stamped_msg = PoseStamped()
                    pose_stamped_msg.header.frame_id = self.reference_frame
                    pose_stamped_msg.pose.position = goal_location_msg
                    pose_stamped_msg.pose.orientation = goal_orientation_msg

                # process task
                if task['task_type'] == 'Move': # nav
                    if goal_actor_msg is not None:
                        agent.actor_goal_publisher_.publish(goal_actor_msg)
                    elif pose_stamped_msg is not None:
                        agent.pose_goal_publisher_.publish(pose_stamped_msg)  
                    else:
                        print('Move task requires target actor or pose.')
                        continue
                else: # pick/drop
                    # approach location
                    if use_default_apporach_location:
                        bool_msg = Bool()
                        bool_msg.data = True
                        agent.set_use_default_approach_publisher_.publish(bool_msg)
                        
                    elif approach_actor is not None and approach_actor != '':
                        approach_actor_msg = String()
                        approach_actor_msg.data = approach_actor
                        agent.set_approach_location_actor_publisher_.publish(approach_actor_msg)

                    elif approach_location is not None and len(approach_location) >= 3:
                        approach_location_msg = PointStamped()
                        approach_location_msg.header.frame_id = self.reference_frame
                        approach_location_msg.point.x = approach_location[0]
                        approach_location_msg.point.y = approach_location[1]
                        approach_location_msg.point.z = approach_location[2]
                        agent.set_approach_location_publisher_.publish(approach_location_msg)
                    
                    if task['task_type'] == 'Pick':
                        if goal_actor_msg is not None:
                            agent.pick_actor_goal_publisher_.publish(goal_actor_msg)
                        elif point_stamped_msg is not None:
                            agent.pick_goal_publisher_.publish(point_stamped_msg)  
                        else:
                            print('Pick task requires target actor or position.')
                            continue
                    elif task['task_type'] == 'Drop':
                        if goal_actor_msg is not None:
                            agent.drop_actor_goal_publisher_.publish(goal_actor_msg)
                        elif pose_stamped_msg is not None:
                            agent.drop_goal_publisher_.publish(pose_stamped_msg)  
                        else:
                            print('Drop task requires target actor or pose.')
                            continue        

def main(args=None):
    rclpy.init(args=args)

    warehouse_client = WarehouseClient('warehouse_client')
    warehouse_client.process_order()

    try:
        rclpy.spin(warehouse_client)
    except KeyboardInterrupt:
        pass

    warehouse_client.destroy_node()

    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass


if __name__ == '__main__':
    main()
