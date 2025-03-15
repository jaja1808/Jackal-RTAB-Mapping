from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare remapping arguments
    odom_remap = LaunchConfiguration('odom_remap', default='/jackal/platform/odom/filtered')
    scan_remap = LaunchConfiguration('scan_remap', default='/scan')
    cmd_vel_remap = LaunchConfiguration('cmd_vel_remap', default='/jackal/cmd_vel')

    # Launch Nav2 for Navigation with remapped topics
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'),
                 'launch',
                 'navigation_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_remap,
            'scan_topic': scan_remap,
            'cmd_vel_topic': cmd_vel_remap,
        }.items(),
    )

    # Create and return the LaunchDescription
    ld = LaunchDescription()

    # Declare the remapping launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument('odom_remap', default_value='/jackal/platform/odom/filtered', description='Remap odometry topic'))
    ld.add_action(DeclareLaunchArgument('scan_remap', default_value='/scan', description='Remap scan topic'))
    ld.add_action(DeclareLaunchArgument('cmd_vel_remap', default_value='/jackal/cmd_vel', description='Remap velocity command topic'))

    # Add the navigation launch
    ld.add_action(navigation_launch)

    return ld

