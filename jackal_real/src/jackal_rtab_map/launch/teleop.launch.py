from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='gnome-terminal --',  # Use gnome-terminal 
            name='teleop',
            remappings=[
                ('cmd_vel', '/jackal/cmd_vel')
            ]
        )
    ])