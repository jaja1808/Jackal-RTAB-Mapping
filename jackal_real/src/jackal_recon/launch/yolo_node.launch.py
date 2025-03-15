from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare the use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Declare the use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Launch the YOLO node
        Node(
            package='jackal_recon',
            executable='yolo.py',
            name='yolo_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])