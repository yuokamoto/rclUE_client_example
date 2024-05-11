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
from example_interfaces.msg import Int32, Float32, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion

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
                'use_default_apporach_location': True
            },
            {
                'task_id': 'task_2',
                'task_type': 'Drop', 
                'goal_actor': 'DropPoint',
                'approach_actor': 'ApproachPoint2'
            }
            
        ]
        # load csv
        # parse csv to dict
        # create task list for agents
        # create pub/sub
    
    def process_order(self):
        print('process_order', len(self.agent_list))
        for agent in self.agent_list:
            rclpy.spin_once(agent, timeout_sec=0.001)
            if agent.task_status == AITaskType.NONE.value and agent.nav_status == AINavigationStatus.IDLE.value:
                #todo check task dependency 
                task = self.task_list[agent.entity_name].pop(0)
                if 'task_type' not in task:
                    print('task_type not found in task {}'.format(task['task_id'] if 'task_id' in task else 'None'))
                    continue
                if task['task_type'] == 'Pick':
                    pass
                elif task['task_type'] == 'Drop':
                    pass
                else task['task_type'] == 'Move':
                    pass
                

            # check task is none and navigation is none
            # if task is none then assign task


        # time.sleep(1)
        

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
