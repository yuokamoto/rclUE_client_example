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
from example_interfaces.msg import Int8, Float32
from geometry_msgs.msg import Pose, Quaternion

from .common import ExternalDeviceClient, ModelNames


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

# Parameter
CONVEYOR_POSE = {'x': 1.0, 'y': 0.0, 'z': 0.5, 'roll': 0.0, 'pitch': 0.0, 'yaw': 1.57}
PAYLOAD_SPAWN_POSE = {'x': 1.0, 'y': 0.0, 'z': 5.5, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
CONVEYOR_SCALE = {'X': 2.5 , 'Y': 1.0, 'Z': 1.0} # it uses capital since X,Y,Z is attribute name of UE
CONVEYOR_SPEED = 1.0
DEBUG = False
ENABLE_WIDGET = True

class ConveyorClient(ExternalDeviceClient):
    def __init__(self, name='conveyor', disable_physics=False):
        super().__init__(name + '_client')
        self.speed_publisher_ = self.create_publisher(Float32, 'set_vel', 10)
        self.mode_publisher_ = self.create_publisher(Int8, 'set_mode', 10)
        self.status_subscription = self.create_subscription(
            Int8,
            'entrances',
            self.entrances_cb,
            10)
        self.status_subscription  # prevent unused variable warning
        self.mode = ConveyorMode.MOVE_TILL_HIT
        self.payload_status = False

        json_parameters = {
            'scale': CONVEYOR_SCALE,
            'speed': CONVEYOR_SPEED,
            'mode': ConveyorMode.MOVE_TILL_HIT.value,
            'enable_widget': ENABLE_WIDGET,
            'disable_physics': disable_physics
        }
        self.spawn_model(CONVEYOR_POSE, ModelNames.CONVEYOR.value, name, name, 'ExternalDevice', json_parameters)
        self.spawn_payload(PAYLOAD_SPAWN_POSE, ModelNames.CUBE.value)
        
    def entrances_cb(self, msg):
        if msg.data:
            speed_cmd = Float32()
            mode_cmd = Int8()
            # move payload until hit the sensor
            if self.mode == ConveyorMode.MOVE_TILL_HIT:
                
                self.mode = ConveyorMode.MOVE_OUT
                
                speed_cmd.data = -CONVEYOR_SPEED
                mode_cmd.data = self.mode.value
                
                self.speed_publisher_.publish(speed_cmd)
                self.mode_publisher_.publish(mode_cmd)

            # move payload until payload move out
            elif self.mode == ConveyorMode.MOVE_OUT:

                self.mode = ConveyorMode.MOVE_TILL_HIT
                
                speed_cmd.data = CONVEYOR_SPEED
                mode_cmd.data = self.mode.value
                
                self.speed_publisher_.publish(speed_cmd)
                self.mode_publisher_.publish(mode_cmd)
                
                time.sleep(2)
                
                self.spawn_payload(PAYLOAD_SPAWN_POSE, ModelNames.CUBE.value)
            
        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
    # def timer_callback(self):
    #     self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    physics_client = ConveyorClient('physics_conveyor', True)
    non_physics_client = ConveyorClient('non_physics_conveyor', False)

    rclpy.spin(client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
