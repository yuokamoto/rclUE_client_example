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
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32, Float32, Int32MultiArray
from geometry_msgs.msg import Pose, Quaternion

from .common import ExternalDeviceClient, ModelNames, array_to_pose_param
from .conveyor_client import ConveyorClient, ConveyorMode

##########################################################################################
# WIP: there is issue to attach component with ROS Spawn
# Conveyor Control client example
#     1. Spawn conveyor with mode=1 and speed=1[m/s], i.e. payload will move and stop at exit
#     2. Spawn Payload
#     3. Wait until payload stop at entrance/exit
#     4. Set mode=0, i.e. paylod will move until move out from conveyor
#     5. Set speed=-1, i.e. conveyor move payload reverse
#     5. Wait until paylod move out.
#     6. Repeat from step 2
##########################################################################################

class SplineConveyorClient(ConveyorClient):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    def ros_api_settings(self):
        super().ros_api_settings()

        # parameters
        # use str since array of array is not supported.
        self.declare_parameter('points', "[\
                                            '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', \
                                            '[1.0, 0.0, 0.0, 0.0, 0.0, 0.785]', \
                                            '[2.0, 2.0, 0.0, 0.0, 0.0, 1.57]'\
                                        ]"
                            )

        self.json_parameters['points'] =  self.parse_points()
    
    def parse_points(self):
        points_str = self.get_parameter('points').value
        points = eval(points_str)
        output = []
        for p_str in points:
            p = eval(p_str)
            if len(p) == 6:
                output.append(array_to_pose_param(p))
            else:
                self.get_logger().info('spline points length should be 6')
        
        return output

def main(args=None):
    rclpy.init(args=args)

    conveyor_client = SplineConveyorClient('conveyor_client')

    rclpy.spin(conveyor_client)

    conveyor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
