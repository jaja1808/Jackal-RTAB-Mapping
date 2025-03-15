# Example:
#   $ ros2 launch jackal_gazebo_sim jackal_world.launch.py 
#
#   SLAM:
#   $ ros2 launch jackal_gazebo_sim realsense_d435.launch.py 
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
        #   'subscribe_scan_cloud':True,
          'sync_queue_size':10000,
          'topic_queue_size':2,
          'approx_sync_max_interval': 0.01,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
        #   'RGBD/NeighborLinkRefining':'True',
        #   'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
        #   'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings = [
        ('/jackal/rgb/image', '/jackal/camera/color/image_raw'),
        ('/jackal/rgb/camera_info', '/jackal/camera/color/camera_info'),
        ('/jackal/depth/image', '/jackal/camera/aligned_depth_to_color/image_raw'),
        ('/jackal/odom', '/jackal/platform/odom/filtered'),
        # ('/jackal/scan_cloud', '/rslidar_points'),
        ('/jackal/scan', '/scan'),
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'), 
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen', namespace='jackal',
            parameters=[{'approx_sync': True, 'use_sim_time': use_sim_time}],
            remappings=remappings),

        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen', namespace='jackal',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),

        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen', namespace='jackal',
            parameters=[parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen', namespace='jackal',
            parameters=[parameters],
            remappings=remappings),
    ])
