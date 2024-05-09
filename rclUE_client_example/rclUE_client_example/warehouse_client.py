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


##########################################################################################
# Character Control client example
#     1. spawn character
#     2. Keep sending random goal if it is idle.
##########################################################################################


class WarehouseClient(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.ros_api_settings()
        self.parse_csv()
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.process_order)

    def ros_api_settings(self):
        pass

    def parse_csv(self):
        pass
        # load csv
        # parse csv to dict
        # create task list for agents
        # create pub/sub
    
    def process_order(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    warehouse_client = WarehouseClient('warehouse_client')

    rclpy.spin(warehouse_client)

    warehouse_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
