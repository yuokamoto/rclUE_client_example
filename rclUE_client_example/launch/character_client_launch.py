from launch import LaunchDescription
from launch_ros.actions import Node

from rclUE_client_example.common import AIMoveMode

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rclUE_client_example',
            name = 'ROSCharacter0',
            namespace = 'manual_chracter',
            executable='character_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': False,
                'mode': AIMoveMode.MANUAL.value,
                'enable_widget': True,
                'spawn_pose': [0.0, -15.0, 0.88, 0.0, 0.0, 1.57],
                'acceptance_radius': 0.1,
                'speed': 5.0,
                'model_name': 'BP_ROSCharacter'
            }]

        ),
        Node(
            package='rclUE_client_example',
            name = 'ROSCharacter1',
            namespace = 'sequence_moving_chracter',
            executable='character_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': True,
                'mode': AIMoveMode.SEQUENCE.value, # move to given goal sequence one by one
                'enable_widget': True,
                'spawn_pose': [5.0, 0.0, 0.88, 0.0, 0.0, 0.0],
                'acceptance_radius': 0.1,
                'speed': 10.0,
                'model_name': 'BP_ROSCharacter',
                # param for mode=1,2,3
                'origin': '[5.0, 0, 0.88, 0.0, 0.0, 0.0]', # origin by coordinate.
                # param for mode=1,2
                'goal_sequence':"[\
                            'MapOrigin', \
                            '[10.0, 0.0,  0.0, 0.0, 0.0, 0.0]', \
                            '[10.0, 10.0, 0.0, 0.0, 0.0, 0.0]', \
                            '[0.0,  10.0, 0.0, 0.0, 0.0, 0.0]' \
                        ]"
            }]
        ),
        Node(
            package='rclUE_client_example',
            name = 'ROSCharacter2',
            namespace = 'random_sequence_moving_chracter',
            executable='character_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': True,
                'mode': AIMoveMode.RANDOM_SEQUENCE.value, # move to given goal sequence randomly
                'enable_widget': True,
                'spawn_pose': [0.0, 0.0, 0.88, 0.0, 0.0, 0.0],
                'acceptance_radius': 0.1,
                'speed': 10.0,
                'model_name': 'BP_ROSCharacter',
                # param for mode=1,2,3
                'origin': 'MapOrigin', # origin by Actor name.
                # param for mode=1,2
                'goal_sequence':"[\
                            '[0.0,   0.0,   0.0, 0.0, 0.0, 0.0]', \
                            '[-10.0, 0.0,   0.0, 0.0, 0.0, 0.0]', \
                            '[-10.0, -10.0, 0.0, 0.0, 0.0, 0.0]', \
                            '[0.0,   -10.0, 0.0, 0.0, 0.0, 0.0]' \
                        ]"
            }]
        ),
        Node(
            package='rclUE_client_example',
            name = 'ROSCharacter3',
            namespace = 'random_area_moving_chracter',
            executable='character_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': True,
                'mode': AIMoveMode.RANDOM_AREA.value, # move randomly inside random_move_bounding_box
                'enable_widget': True,
                'spawn_pose': [10.0, 0.0, 0.88, 0.0, 0.0, 0.0],
                'acceptance_radius': 0.1,
                'speed': 2.0,
                'model_name': 'BP_ROSCharacter',
                # param for mode=1,2,3
                'origin': 'MapOrigin', # origin by Actor name.
                # param for mode=3
                'random_move_bounding_box': [5.0, 5.0, 0.0]
            }]
        )
    ])