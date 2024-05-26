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
import csv
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from example_interfaces.msg import Int32, Float32, Int32MultiArray, Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Pose, Point

from .common import ExternalDeviceClient, ModelNames, AIMoveMode, array_to_vector_param, array_to_pose_param

from .ai_actor_client import AIControlledActorClient, AITaskType, AINavigationStatus

##########################################################################################
# Character Control client example
#     1. spawn character
#     2. Keep sending random goal if it is idle.
##########################################################################################

class PayloadClient(Node):
    def __init__(self, name, model, spawn=False, **kwargs):
        super().__init__(name, **kwargs)
        self.payload = None
        self.payload_sub = None
        self.children_actor_list = {}

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.children_actor_list_subscription = self.create_subscription(
            String,
            'children_actor_list',
            self.children_actor_list_cb,
            qos_profile = qos_profile)

        self.children_actor_list_subscription  # prevent unused variable warning

        # todo spawn payload with name and model
        if  spawn:
            pass
    
    def children_actor_list_cb(self, msg):
        key_value = msg.data.split(',')
        for kv in key_value:
            if ':' in kv:
                key, value = kv.split(':')
                self.children_actor_list[key] = value
    
    def get_child_actor(self, name):
        return self.children_actor_list[name]
        

class WarehouseClient(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.task_list = {}
        self.agent_list = []
        self.payload_list = {}
        # todo handle non agent list tasks

        self.ros_api_settings()
        self.parse_csv()

        # todo update if there is new payload
        for payload in self.payload_list:
            rclpy.spin_once(self.payload_list[payload], timeout_sec=0.001)

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.process_order)

    def update_payload_list(self):
        topics = self.get_topic_names_and_types()
        for topic in topics:
            if 'children_actor_list' in topic[0]:
                name = topic[0].split('/')[1]
                if name not in self.payload_list:
                    self.payload_list[name] = PayloadClient('payload_client', '', spawn=False, namespace=name)
                    print('add paylaod:', name)
        
    def ros_api_settings(self):
        self.update_payload_list()        

    def parse_csv(self):
        # read agent list -> create agent_list and initialize task_list
        with open('/home/rr/Downloads/testdata(AgentList).csv', "r", encoding="utf-8") as csvfile:
            reader = csv.DictReader(csvfile)
            for d in reader:
                # todo spawn agent
                if d['spawn']:
                    pass

                self.agent_list.append(
                    {
                        'agent_id': d['agent_id'],
                        'agent': AIControlledActorClient('AIControlledActorClient', spawn=False, namespace=d['agent_id']),
                        'current_task': ''
                    }
                )

                self.task_list[d['agent_id']] = []

        # read task_list -> append and change defalut value of dependency, location, orienation, dependency
        with open('/home/rr/Downloads/testdata(TaskList).csv', "r", encoding="utf-8") as csvfile:
            reader = csv.DictReader(csvfile)
            
            def check_value(key, task):
                return task[key] if key in task and task[key] != '' else None            
            
            # udpate default value of optional keys
            keys = [
                'use_default_apporach_location', 
                'goal_location', 
                'goal_orientation', 
                'goal_actor', 
                'point_in_goal_actor',
                'approach_location', 
                'approach_actor',
                'dependency'
            ]
            for d in reader:
                if not d['agent_id'] :
                    continue

                for key in keys:
                    d[key] = check_value(key, d)

                if d['meta'] != '':
                    d['meta'] = json.dumps(json.loads(d['meta']))

                d['completed'] = False            

                self.task_list[d['agent_id']].append(d)

        # print(self.task_list)

        # self.task_list['BP_ROSCharacter14'] = [
        #     {
        #         'task_id': 'task_1',
        #         'task_type': 'Pick', 
        #         # 'goal_location': [],
        #         # 'goal_orientation': []
        #         'goal_actor': 'BP_Cart',
        #         # 'approach_location':[]
        #         # 'approach_actor':
        #         'use_default_apporach_location': True,
        #         'dependency': [],
        #         'completed': False
        #     },
        #     {
        #         'task_id': 'task_2',
        #         'task_type': 'Drop', 
        #         'goal_actor': 'DropPoint',
        #         'approach_actor': 'ApproachPoint2',
        #         'dependency': [],
        #         'completed': False
        #     }
            
        # ]
    
    def get_agent_task_by_id(self, agent_name, task_id):
        for task in self.task_list[agent_name]:
            if task['task_id'] == task_id:
                return task
        return None

    def get_task_by_id(self, task_id):
        for agent_name in self.task_list:
            res = self.get_agent_task_by_id(agent_name, task_id)
            if res is not None:
                return res
        return None

    def get_top_uncompleted_agnet_task(self, agent_name):
        for task in self.task_list[agent_name]:
            if not task['completed']:
                return task
        return None
    
    def update_agent_task_status(self, agent_name, task_id, status):
        task = self.get_agent_task_by_id(agent_name, task_id)
        if task is not None:
            task['completed'] = status
        else:
            print('task not found with id {}'.format(task_id))
    
    def update_task_status(self, task_id, status):
        for agent_name in self.task_list:
            self.update_agent_task_status(agent_name, task_id, status)

    def get_agent_task_dependency(self, agent_name, task_id):
        task = self.get_agent_task_by_id(agent_name, task_id)
        if task is not None:
            return task['dependency']
        return None
    
    def get_task_dependency(self, task_id):
        for agent_name in self.task_list:
            dependency = self.get_agent_task_dependency(agent_name, task_id)
            if dependency is not None:
                return dependency
        return None

    def check_agent_task_dependency(self, agent_name, task_id):
        dependency = self.get_agent_task_dependency(agent_name, task_id)
        if dependency is not None:
            for dep in dependency.split(','):
                task = self.get_task_by_id(dep)
                if task is not None and not task['completed']:
                    return False
        return True
    
    def check_task_dependency(self, task_id):
        dependency = self.get_task_dependency(task_id)
        if dependency is not None:
            for dep in dependency.split(','):
                task = self.get_task_by_id(dep)
                if task is not None and not task['completed']:
                    return False
        return True

    def process_order(self):

        # print('process_order')
        self.update_payload_list()
        for ag in self.agent_list:
            agent = ag['agent']
            rclpy.spin_once(agent, timeout_sec=0.001)
            rclpy.spin_once(agent, timeout_sec=0.001)

            if agent.task_status == 0 and agent.nav_status == AINavigationStatus.IDLE.value and len(self.task_list[agent.entity_name]) > 0:
                
                print('[{}] check next task'.format(agent.entity_name))

                # update last task status
                self.update_agent_task_status(agent.entity_name, ag['current_task'], True)
                
                task = self.get_top_uncompleted_agnet_task(agent.entity_name)
                print(task)
                
                # task sanity check
                if task is None:
                    continue 

                is_dependent_task = not self.check_agent_task_dependency(agent.entity_name, task['task_id'])
                if is_dependent_task:
                    print('task {} is dependent'.format(task['task_id']))
                    continue
                elif 'task_type' not in task and task['task_type'] not in AITaskType.__members__:
                    print('task_type not found in task {}'.format(task['task_id'] if 'task_id' in task else 'None'))
                    continue
                
                print('[{}] send next task'.format(agent.entity_name))

                ag['current_task'] = task['task_id']
                agent.task_status = task['task_type']

                # parse param
                use_default_apporach_location = task['use_default_apporach_location']
                goal_loction = task['goal_location']
                goal_orientation = task['goal_orientation']
                goal_actor = task['goal_actor']
                approach_location = task['approach_location']
                approach_actor = task['approach_actor']
                point_in_goal_actor = task['point_in_goal_actor']
                meta_data = task['meta']
            
                # parse goal location
                goal_actor_msg = goal_location_msg = goal_orientation_msg = point_stamped_msg = pose_stamped_msg = None
                if goal_actor is not None:
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
                if task['task_type'] == AITaskType.MOVE.value: # nav
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
                    
                    if task['task_type'] == AITaskType.PICK.value:
                        if goal_actor_msg is not None:
                            agent.pick_actor_goal_publisher_.publish(goal_actor_msg)
                        elif point_stamped_msg is not None:
                            agent.pick_goal_publisher_.publish(point_stamped_msg)  
                        else:
                            print('Pick task requires target actor or position.')
                            continue
                    elif task['task_type'] == AITaskType.DROP.value:
                        if goal_actor_msg is not None:
                            # print('test\n', point_in_goal_actor, goal_actor, goal_actor in self.payload_list, '\n\n')
                            # if point_in_goal_actor is not None and goal_actor in self.payload_list:
                            #     print(
                            #         point_in_goal_actor in self.payload_list[goal_actor].children_actor_list, 
                            #         self.payload_list[goal_actor].children_actor_list[point_in_goal_actor]
                            #     )
                            # print(point_in_goal_actor, goal_actor, 
                            #     point_in_goal_actor in self.payload_list[goal_actor].children_actor_list, 
                            #     self.payload_list[goal_actor].children_actor_list[point_in_goal_actor]
                            # )
                            if point_in_goal_actor is not None and \
                                goal_actor in self.payload_list and \
                                point_in_goal_actor in self.payload_list[goal_actor].children_actor_list:
                                goal_actor_msg.data = self.payload_list[goal_actor].children_actor_list[point_in_goal_actor]
                            agent.drop_actor_goal_publisher_.publish(goal_actor_msg)
                        elif pose_stamped_msg is not None:
                            agent.drop_goal_publisher_.publish(pose_stamped_msg)  
                        else:
                            print('Drop task requires target actor or pose.')
                            continue    
                    elif task['task_type'] == AITaskType.GENERAL_ACTION.value:    
                        meta_msg = String()
                        meta_msg.data = meta_data
                        agent.general_action_publisher.publish(meta_msg)

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
