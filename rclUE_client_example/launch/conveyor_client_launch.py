from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rclUE_client_example',
            name = 'client',
            namespace = 'physics_conveyor',
            executable='conveyor_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': False,
                'enable_widget': True,
                'disable_physics': False,
                'mode': 0,
                'size': [2.0, 0.5, 0.1],
                'spawn_pose': [0.0, 0.0, 0.5, 0.0, 0.0, 1.57],
                'payload_spawn_pose': [0.0, 0.0, 3.0, 0.0, 0.0, 0.0]
            }]

        ),
        Node(
            package='rclUE_client_example',
            name = 'client',
            namespace = 'non_physics_conveyor',
            executable='conveyor_client',
            output='screen',
            
            parameters=[{
                'debug': False,
                'enable_widget': True,
                'disable_physics': True,
                'mode': 0,
                'size': [3.0, 0.5, 0.1],
                'spawn_pose': [0.0, 3.0, 0.5, 0.0, 0.0, 0.0],
                'payload_spawn_pose': [0.0, 3.0, 3.0, 0.0, 0.0, 0.0]
            }]
        )
    ])