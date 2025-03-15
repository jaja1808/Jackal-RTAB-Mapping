from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():

    # Launch 3D lidar
    rslidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rslidar_sdk'),
                    'launch',
                    'start.py']
            )
        ),
    ) 
    
    # Launch 2D lidar
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_a3_launch.py']
            )
        ),
    ) 
    
    # Launch Realsense Camera
    Camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('jackal_bringup'),
                    'launch',
                    'realsense_camera.launch.py']
            )
        ),
    ) 

    # Create and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(rslidar)
    ld.add_action(rplidar)
    ld.add_action(Camera)

    return ld
