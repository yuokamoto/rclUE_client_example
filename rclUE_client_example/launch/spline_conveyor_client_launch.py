from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rclUE_client_example',
            name = 'client',
            namespace = 'spline_physics_conveyor',
            executable='spline_conveyor_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': True,
                'enable_widget': True,
                'disable_physics': False,
                'mode': 0,
                'size': [1.0, 1.0, 1.0],
                'spawn_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 1.57],
                'payload_spawn_pose': [0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
                'model_name': 'BP_BeltSplineConveyor',
                'points': "['[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', \
                            '[1.0, 0.0, 0.0, 0.0, 0.0, -0.785]', \
                            '[2.0, 1.0, 0.0, 0.0, 0.0, -0.785]', \
                            '[2.0, 2.0, 0.0, 0.0, 0.0, -1.57]' \
                        ]"
                # use str since array of array is not supported.
                # x, y, z, roll, pitch, yaw
            }] 

        ),
        Node(
            package='rclUE_client_example',
            name = 'client',
            namespace = 'spline_non_physics_conveyor',
            executable='spline_conveyor_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': False,
                'enable_widget': True,
                'disable_physics': True,
                'mode': 0,
                'size': [1.0, 1.0, 1.0],
                'spawn_pose': [0.0, 3.0, 0.0, 0.0, 0.0, 0.0],
                'payload_spawn_pose': [0.0, 3.0, 3.0, 0.0, 0.0, 0.0],
                'model_name': 'BP_RollerSplineConveyor',
                'points': "['[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', \
                            '[2.0, 0.0, 0.0, 0.0, 0.0, 0.785]', \
                            '[4.0, -2.0, 0.0, 0.0, 0.0, 0.785]', \
                            '[4.0, -4.0, 0.0, 0.0, 0.0, 1.57]' \
                        ]"

            }]
        )
    ])