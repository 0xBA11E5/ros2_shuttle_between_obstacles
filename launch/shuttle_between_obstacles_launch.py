from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shuttle_between_obstacles',
            executable='follow_line',

            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],

            parameters=[
                {'boundary_left': 92},
                {'boundary_right': 198},
                {'threshold_line': 102}
            ]
        ),
        Node(
            package='shuttle_between_obstacles',
            executable='follow_object',

            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
        Node(
            package='shuttle_between_obstacles',
            executable='shuttle_between_obstacles',

            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
    ])
