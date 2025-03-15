from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    rviz_path = os.path.join('/home/administrator/jackal_ws/src/jackal_rtab_map/rviz/',
    'rviz_jackal.rviz',
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                ('/tf', '/jackal/tf'),
                ('/tf_static', '/jackal/tf_static')
            ],
            
            arguments=["-d", rviz_path],

            parameters=[{
                'use_sim_time': False  # Set to True if you're using simulation time
            }]
        )
    ])
