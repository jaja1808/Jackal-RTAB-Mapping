from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='rslidar_sdk',  # The new namespace
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            output='screen',
        ),
    ])