# # Example:
# #   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
# #   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
# #
# #   SLAM:
# #   $ ros2 launch rtabmap_examples vlp16.launch.py


# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time')
#     deskewing = LaunchConfiguration('deskewing')
    
#     return LaunchDescription([

#         # Launch arguments
#         DeclareLaunchArgument(
#             'use_sim_time', default_value='true',
#             description='Use simulation (Gazebo) clock if true'),
        
#         DeclareLaunchArgument(
#             'deskewing', default_value='true',
#             description='Enable lidar deskewing'),
          
#         # Nodes to launch
#         Node(
#             package='rtabmap_odom', executable='icp_odometry', output='screen',
#             parameters=[{
#               'frame_id':'velodyne',
#               'odom_frame_id':'odom',
#               'wait_for_transform':0.2,
#               'expected_update_rate':15.0,
#               'deskewing':deskewing,
#               'use_sim_time':use_sim_time,
#             }],
#             remappings=[
#               ('scan_cloud', '/velodyne/mid/points')
#             ],
#             arguments=[
#               'Icp/PointToPlane', 'true',
#               'Icp/Iterations', '10',
#               'Icp/VoxelSize', '0.1',
#               'Icp/Epsilon', '0.001',
#               'Icp/PointToPlaneK', '20',
#               'Icp/PointToPlaneRadius', '0',
#               'Icp/MaxTranslation', '2',
#               'Icp/MaxCorrespondenceDistance', '1',
#               'Icp/Strategy', '1',
#               'Icp/OutlierRatio', '0.7',
#               'Icp/CorrespondenceRatio', '0.01',
#               'Odom/ScanKeyFrameThr', '0.4',
#               'OdomF2M/ScanSubtractRadius', '0.1',
#               'OdomF2M/ScanMaxSize', '15000',
#               'OdomF2M/BundleAdjustment', 'false',
#             ]),
            
#         Node(
#             package='rtabmap_util', executable='point_cloud_assembler', output='screen',
#             parameters=[{
#               'max_clouds':10,
#               'fixed_frame_id':'',
#               'use_sim_time':use_sim_time,
#             }],
#             remappings=[
#               ('cloud', 'odom_filtered_input_scan')
#             ]),
            
#         Node(
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[{
#               'frame_id':'velodyne',
#               'subscribe_depth':False,
#               'subscribe_rgb':False,
#               'subscribe_scan_cloud':True,
#               'approx_sync':False,
#               'wait_for_transform':0.2,
#               'use_sim_time':use_sim_time,
#             }],
#             remappings=[
#               ('scan_cloud', '/velodyne/mid/points')
#             ],
#             arguments=[
#               '-d', # This will delete the previous database (~/.ros/rtabmap.db)
#               'RGBD/ProximityMaxGraphDepth', '0',
#               'RGBD/ProximityPathMaxNeighbors', '1',
#               'RGBD/AngularUpdate', '0.05',
#               'RGBD/LinearUpdate', '0.05',
#               'RGBD/CreateOccupancyGrid', 'false',
#               'Mem/NotLinkedNodesKept', 'false',
#               'Mem/STMSize', '30',
#               'Mem/LaserScanNormalK', '20',
#               'Reg/Strategy', '1',
#               'Icp/VoxelSize', '0.1',
#               'Icp/PointToPlaneK', '20',
#               'Icp/PointToPlaneRadius', '0',
#               'Icp/PointToPlane', 'true',
#               'Icp/Iterations', '10',
#               'Icp/Epsilon', '0.001',
#               'Icp/MaxTranslation', '3',
#               'Icp/MaxCorrespondenceDistance', '1',
#               'Icp/Strategy', '1',
#               'Icp/OutlierRatio', '0.7',
#               'Icp/CorrespondenceRatio', '0.2',
#             ]), 
     
#         Node(
#             package='rtabmap_viz', executable='rtabmap_viz', output='screen',
#             parameters=[{
#               'frame_id':'velodyne',
#               'odom_frame_id':'odom',
#               'subscribe_odom_info':True,
#               'subscribe_scan_cloud':True,
#               'approx_sync':False,
#               'use_sim_time':use_sim_time,
#             }],
#             remappings=[
#                ('scan_cloud', '/velodyne/mid/points')
#             ]),
#     ])
    
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')
    
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable lidar deskewing'),
          
        # ICP Odometry Node (Velodyne for scan matching)
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':'velodyne',
              'odom_frame_id':'odom',
              'wait_for_transform':0.1,
              'expected_update_rate':1.0,
              'deskewing':deskewing,
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('scan_cloud', '/velodyne/mid/points')  # Remapping Velodyne point cloud topic
            ],
            arguments=[
              'Icp/PointToPlane', 'true',
              'Icp/Iterations', '10',
              'Icp/VoxelSize', '0.1',
              'Icp/Epsilon', '0.001',
              'Icp/PointToPlaneK', '20',
              'Icp/PointToPlaneRadius', '0',
              'Icp/MaxTranslation', '2',
              'Icp/MaxCorrespondenceDistance', '1',
              'Icp/Strategy', '1',
              'Icp/OutlierRatio', '0.7',
              'Icp/CorrespondenceRatio', '0.01',
              'Odom/ScanKeyFrameThr', '0.4',
              'OdomF2M/ScanSubtractRadius', '0.1',
              'OdomF2M/ScanMaxSize', '15000',
              'OdomF2M/BundleAdjustment', 'false',
            ]),
            
        # Point Cloud Assembler Node
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
              'max_clouds':15,
              'fixed_frame_id':'base_link',
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('cloud', '/rtabmap/cloud')  # Using Velodyne point cloud topic
            ]),
            
        # SLAM Node (RTAB-Map)
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':'velodyne',
              'subscribe_depth':False,
              'subscribe_rgb':False,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'wait_for_transform':0.2,
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('scan_cloud', '/velodyne/mid/points'),  # Velodyne scan cloud topic
              ('rgb/image', '/camera/image_raw'),  # Camera image remap
              ('rgb/camera_info', '/camera/camera_info'),  # Camera info remap
              ('depth/image', '/camera/depth/image_raw'),  # Depth image remap
              ('depth/camera_info', '/camera/depth/camera_info'),  # Depth camera info remap
              ('cloud', '/camera/points')  # Camera points remap
            ],
            arguments=[
              '-d',  # Delete the previous database (~/.ros/rtabmap.db)
              'RGBD/ProximityMaxGraphDepth', '0',
              'RGBD/ProximityPathMaxNeighbors', '1',
              'RGBD/AngularUpdate', '0.05',
              'RGBD/LinearUpdate', '0.05',
              'RGBD/CreateOccupancyGrid', 'false',
              'Mem/NotLinkedNodesKept', 'false',
              'Mem/STMSize', '30',
              'Mem/LaserScanNormalK', '20',
              'Reg/Strategy', '1',
              'Icp/VoxelSize', '0.1',
              'Icp/PointToPlaneK', '20',
              'Icp/PointToPlaneRadius', '0',
              'Icp/PointToPlane', 'true',
              'Icp/Iterations', '10',
              'Icp/Epsilon', '0.001',
              'Icp/MaxTranslation', '3',
              'Icp/MaxCorrespondenceDistance', '1',
              'Icp/Strategy', '1',
              'Icp/OutlierRatio', '0.7',
              'Icp/CorrespondenceRatio', '0.2',
            ]), 
     
        # RTAB-Map Visualization Node
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
              'frame_id':'velodyne',
              'odom_frame_id':'odom',
              'subscribe_odom_info':True,
              'subscribe_scan_cloud':False,
              
              'approx_sync':True,
              'use_sim_time':use_sim_time,
            }],
            remappings=[
               ('scan_cloud', '/velodyne/mid/points'),  # Velodyne scan cloud topic
               ('rgb/image', '/camera/image_raw'),  # Camera image remap
               ('depth/image', '/camera/depth/image_raw')  # Depth image remap
            ]),
    ])
    
