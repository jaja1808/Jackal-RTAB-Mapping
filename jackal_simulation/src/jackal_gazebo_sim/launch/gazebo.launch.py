from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
    DeclareLaunchArgument('spawn_x', default_value='0.0'),
    DeclareLaunchArgument('spawn_y', default_value='0.0'),
    DeclareLaunchArgument('spawn_z', default_value='0.0'),
    DeclareLaunchArgument('spawn_R', default_value='0.0'),
    DeclareLaunchArgument('spawn_P', default_value='0.0'),
    DeclareLaunchArgument('spawn_Y', default_value='0.8'),                      
]


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('jackal_description_sim')).
                                                    parent.resolve())])


    # Launch args
    world_path = LaunchConfiguration('world_path')
    

    # Get URDF via xacro
    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control_sim'), 'config', 'control.yaml']
    )     
    
    # Get URDF via xacro
    robot_description_command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description_sim'), 'urdf', 'jackal.urdf.xacro']
            ),
            ' ',
            'is_sim:=true',
            ' ',
            'gazebo_controllers:=',
            config_jackal_velocity_controller,
        ]
    
    yolo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('jackal_recon'),
                     'launch',
                     'yolo_node.launch.py']
                )
            ),
        )
     
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_ray_sensor.so'
             '--verbose',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_jackal',
        arguments=['-entity','jackal',
                   '-topic','robot_description',
                    '-x', LaunchConfiguration('spawn_x'),
                    '-y', LaunchConfiguration('spawn_y'),
                    '-z', LaunchConfiguration('spawn_z'),
                    '-R', LaunchConfiguration('spawn_R'),
                    '-P', LaunchConfiguration('spawn_P'),
                    '-Y', LaunchConfiguration('spawn_Y'),
        ],         
        output='screen',
        )
    
    # rviz node
    rviz_path = os.path.join('./src/jackal_gazebo_sim/rviz',
        'rviz_jackal.rviz',
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        name='jackal_rviz',
        arguments=["-d", rviz_path],
        parameters=[{'use_sim_time': True}],
    )
    
    # Launch jackal_control_sim/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jackal_control_sim'), 'launch', 'teleop_base.launch.py'])))
    
    # Launch jackal_control_sim/teleop_base.launch.py which is various ways to tel

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(yolo_launch)   
    # ld.add_action(launch_jackal_control)
    ld.add_action(launch_jackal_teleop_base)
    ld.add_action(spawn_robot)
    ld.add_action(rviz_node)

    return ld
