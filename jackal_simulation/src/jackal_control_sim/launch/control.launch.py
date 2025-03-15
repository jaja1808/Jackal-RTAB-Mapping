from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Configs
    config_jackal_ekf = PathJoinSubstitution(
        [FindPackageShare('jackal_control_sim'),
         'config',
         'localization.yaml'],
    )

    config_imu_filter = PathJoinSubstitution(
        [FindPackageShare('jackal_control_sim'),
         'config',
         'imu_filter.yaml'],
    )

    # Localization
    localization_group_action = GroupAction([
        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_jackal_ekf],
            remappings=[('/odom', '/odom_filtered')]
        ),

        # Madgwick Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[config_imu_filter]
        )
    ])

    ld = LaunchDescription()
    # ld.add_action(use_sim_time_arg)
    ld.add_action(localization_group_action)
   
    return ld
