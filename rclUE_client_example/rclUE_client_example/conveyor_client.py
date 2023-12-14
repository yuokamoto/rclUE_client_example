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
from example_interfaces.msg import Int8, Float32
from geometry_msgs.msg import Pose, Quaternion

from .common import ExternalDeviceClient, ModelNames


##########################################################################################
# Conveyor Control client example
#     1. Spawn conveyor with mode=1 and speed=100, i.e. payload will move and stop at exit
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

# Parameter
CONVEYOR_POSE = {'x': 1.0, 'y': 0.0, 'z': 0.5, 'roll': 0.0, 'pitch': 0.0, 'yaw': 1.57}
PAYLOAD_SPAWN_POSE = {'x': 1.0, 'y': 0.0, 'z': 5.5, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
CONVEYOR_SCALE = {'X': 2.5 , 'Y': 1.0, 'Z': 1.0} # it uses capital since X,Y,Z is attribute name of UE
CONVEYOR_SPEED = 1.0
DEBUG = False
ENABLE_WIDGET = True

class ConveyorClient(ExternalDeviceClient):

    def __init__(self):
        super().__init__('conveyor_client')
        self.speed_publisher_ = self.create_publisher(Float32, 'set_vel', 10)
        self.mode_publisher_ = self.create_publisher(Int8, 'set_mode', 10)
        self.status_subscription = self.create_subscription(
            Int8,
            'entrances',
            self.entrances_cb,
            10)
        self.status_subscription  # prevent unused variable warning
        self.mode = 1
        self.payload_status = False
        self.payload_id = 0

        json_parameters = {
            'scale': CONVEYOR_SCALE,
            'speed': CONVEYOR_SPEED,
            'mode': ConveyorMode.MOVE_TILL_HIT.value,
            'enable_widget': ENABLE_WIDGET
        }
        self.spawn_model(CONVEYOR_POSE, ModelNames.CONVEYOR.value, '', 'test_conveyor', 'ExternalDevice', json_parameters)
        self.spawn_model(PAYLOAD_SPAWN_POSE, ModelNames.CUBE.value, '', 'payload'+str(self.payload_id), 'Payload', {})
        self.payload_id += 1
        
    def entrances_cb(self, msg):
        if enetrance:
            # move payload until hit the sensor
            if self.mode == ConveyorMode.MOVE_TILL_HIT:
                self.mode = ConveyorMode.MOVE_OUT
                self.speed_publisher_(-CONVEYOR_SPEED)
                self.mode_publisher_(self.mode)
            # move payload until payload move out
            elif self.mode == ConveyorMode.MOVE_OUT:
                self.mode = ConveyorMode.MOVE_TILL_HIT
                self.speed_publisher_(CONVEYOR_SPEED)
                self.mode_publisher_(self.mode)
                time.sleep(2)
                self.delete_model(self.payload_id)
                self.spawn_model(PAYLOAD_SPAWN_POSE, ModelNames.CUBE.value, '', 'payload'+str(self.payload_id), 'Payload', {})
                self.payload_id += 1

            
        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
    # def timer_callback(self):
    #     self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    client = ConveyorClient()

    rclpy.spin(client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
