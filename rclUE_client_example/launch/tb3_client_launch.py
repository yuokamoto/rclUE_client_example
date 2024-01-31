from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rclUE_client_example',
            name = 'ROSTurtlebotBurger',
            namespace = '', #'tb3_burger',
            executable='tb3_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': False,
                'enable_widget': True,
                'spawn_pose': [-0.5, -0.5, 0.0, 0.0, 0.0, 0.0],
                'model_name': 'BP_TurtlebotBurger',
                'map_origin': 'map',
                'goal_sequence':"[\
                            '[2.5,   2.5, 0.0, 0.0, 0.0, 0.0]', \
                            '[-0.5, -0.5, 0.0, 0.0, 0.0, 0.0]' \
                        ]"
            }]

        )
    ])