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

from .common import ExternalDeviceClient, ModelNames, array_to_size_param


##########################################################################################
# Conveyor Control client example
#     1. Spawn conveyor with mode=1 and speed=1[m/s], i.e. payload will move and stop at exit
#     2. Spawn Payload
#     3. Wait until payload stop at entrance/exit
#     4. Set mode=0, i.e. paylod will move until move out from conveyor
#     5. Set speed=-1, i.e. conveyor move payload reverse
#     5. Wait until paylod move out.
#     6. Repeat from step 2
##########################################################################################

from enum import Enum
class ConveyorMode(Enum):
    MOVE_OUT = 0
    MOVE_TILL_HIT = 1

class ConveyorClient(ExternalDeviceClient):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # parameters
        self.declare_parameter('speed', 1.0)

        # pub/sub
        self.speed_publisher_ = self.create_publisher(Float32, 'set_vel', 10)
        self.mode_publisher_ = self.create_publisher(Int32, 'set_mode', 10)
        self.status_subscription = self.create_subscription(
            Int32MultiArray,
            'entrance',
            self.entrances_cb,
            10)
        self.status_subscription  # prevent unused variable warning

        # default values
        self.mode = ConveyorMode.MOVE_TILL_HIT
        self.payload_status = False
        self.speed = self.get_parameter('speed').value

        # spawn 
        json_parameters = {
            'size': self.parse_size_param(),
            'speed': self.speed,
            'debug': self.get_parameter('debug').value,
            'mode': ConveyorMode.MOVE_TILL_HIT.value,
            'enable_widget': self.get_parameter('enable_widget').value,
            'disable_physics': self.get_parameter('disable_physics').value
        }
        namespace = self.get_namespace()
        entity_name = namespace[1:len(namespace)]
        self.spawn_self(self.get_parameter('spawn_pose').value, entity_name, entity_name, '', json_parameters)
        self.spawn_payload(ModelNames.PHYSICS_CUBE.value)
        
    def entrances_cb(self, msg):

        entrance0 = msg.data[0]
        entrance1 = msg.data[1]
        if entrance0:
            speed_cmd = Float32()
            mode_cmd = Int32()
                
            # move payload until payload move out
            self.mode = ConveyorMode.MOVE_OUT
            
            speed_cmd.data = -self.speed
            mode_cmd.data = self.mode.value
            
            self.speed_publisher_.publish(speed_cmd)
            self.mode_publisher_.publish(mode_cmd)

        if entrance1:
            speed_cmd = Float32()
            mode_cmd = Int32()

            time.sleep(2)

            # move payload until hit the sensor
            self.mode = ConveyorMode.MOVE_TILL_HIT
            
            speed_cmd.data = self.speed
            mode_cmd.data = self.mode.value

            self.speed_publisher_.publish(speed_cmd)
            self.mode_publisher_.publish(mode_cmd)                
            self.spawn_payload(ModelNames.PHYSICS_CUBE.value)

def main(args=None):
    rclpy.init(args=args)

    conveyor_client = ConveyorClient('conveyor_client')

    rclpy.spin(conveyor_client)

    conveyor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
