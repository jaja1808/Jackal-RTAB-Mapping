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

    # Declaration of Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_file = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo_sim'),
        'worlds',
        'cafe.world'],
    )

    # Launch other launch files for Jackal funciton and world
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo_sim'),
        'launch',
        'gazebo.launch.py'],
    )

    Localisation = PathJoinSubstitution(
        [FindPackageShare('jackal_control_sim'),
        'launch',
        'control.launch.py'],
    )

    # Launch the RTAB Mapping nodes from this launch file
    rtab_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('jackal_gazebo_sim'),
                    'launch',
                    'rtab_map.launch.py']
            )
        ),
    ) 

    # Launch Nav2 for Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Parsing the Variables in the launch files
    Localisation_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([Localisation]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file,
                          'spawn_x': '1.0',
                          'spawn_y': '2.0',
                          'spawn_z': '0.0',
                          'spawn_R': '0.0',
                          'spawn_P': '0.0',
                          'spawn_Y': '0.0',
                          }.items(),
    )

    # Path to URDF model
    robot_dir = get_package_share_directory("jackal_description_sim")
    model_file = os.path.join(robot_dir, "urdf", "jackal.urdf.xacro")

    # Process URDF file using xacro
    robot_description = xacro.process_file(model_file)

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Define the Node for Jackal description publish
    robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description.toxml()}]
    )

    # Create and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(robot)
    ld.add_action(rtab_map)
    ld.add_action(navigation_launch)
    ld.add_action(gazebo_sim)
    ld.add_action(Localisation_sim)

    return ld
