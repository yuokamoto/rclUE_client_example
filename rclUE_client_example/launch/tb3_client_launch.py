import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    print(os.path.join(get_package_share_directory('rclUE_client_example'), 'config', 'Turtlebot3_benchmark.yaml'))

    return LaunchDescription([
        Node(
            package='rclUE_client_example',
            name = 'ROSTurtlebotBurger',
            namespace = '/', #'tb3_burger',
            executable='tb3_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot_name': 'tb3_burger',
                'debug': False,
                'enable_widget': True,
                'spawn_pose': [-0.5, -0.5, 0.0, 0.0, 0.0, 0.0],
                'model_name': 'BP_TurtlebotWaffle',
                'map_origin': 'map',
                'goal_sequence':"[\
                            '[2.5,   2.5, 0.0, 0.0, 0.0, 0.0]', \
                            '[-0.5, -0.5, 0.0, 0.0, 0.0, 0.0]' \
                        ]"
            }]

        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/tb3_simulation_launch.py']),
            launch_arguments={
                'use_simulator': 'False',
                'map': os.path.join(get_package_share_directory('rclUE_client_example'), 'config', 'Turtlebot3_benchmark.yaml')
            }.items()           
        )
    ])