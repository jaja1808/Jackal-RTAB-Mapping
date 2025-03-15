#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <ultralytics_ros/msg/yolo_result.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

// Define the synchronization policy for subscribing to camera info, point cloud, and YOLO results
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2,
                                                        ultralytics_ros::YoloResult>
    ApproximateSyncPolicy;

// Class for tracking objects using point clouds
class TrackerWithCloudNode : public rclcpp::Node
{
private:
    // ROS2 publishers for various types of messages
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_cloud_pub_; // Publisher for publishing point cloud data (detections)
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_pub_; // Publisher for publishing 3D object detection results
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // Publisher for visualization markers in RViz

    // ROS2 time object to store the time of the last callback
    rclcpp::Time last_call_time_; // Keeps track of the time of the last callback function execution

    // Message filter subscribers to handle incoming messages for camera info, point cloud, and YOLO results
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_; // Subscriber for camera info messages
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_; // Subscriber for point cloud data from a LiDAR sensor
    message_filters::Subscriber<ultralytics_ros::msg::YoloResult> yolo_result_sub_; // Subscriber for YOLO detection results

    // Synchronizer to ensure messages from different topics (CameraInfo, PointCloud2, YoloResult) are synchronized
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_; // Synchronizer for time-aligning multiple incoming messages

    // Transform buffer and listener to handle coordinate frame transformations in ROS2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // Buffer for storing transform data
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // Listener to monitor transform data in real-time

    // Camera model to project 2D images to 3D space
    image_geometry::PinholeCameraModel cam_model_; // Pinhole camera model used for projecting camera images into 3D space

    // ROS topics for incoming and outgoing data
    std::string camera_info_topic_; // Topic name for camera info
    std::string lidar_topic_; // Topic name for point cloud (LiDAR) data
    std::string yolo_result_topic_; // Topic name for YOLO detection results
    std::string yolo_3d_result_topic_; // Topic name for 3D YOLO detection results

    // Parameters for point cloud clustering and voxel grid filtering
    float cluster_tolerance_; // Tolerance for clustering points in the point cloud
    float voxel_leaf_size_; // Leaf size for down-sampling the point cloud using a voxel grid filter
    int min_cluster_size_; // Minimum number of points required to form a cluster
    int max_cluster_size_; // Maximum number of points allowed in a cluster

public:
    TrackerWithCloudNode(); // Constructor for the tracker node

    void syncCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
                    const ultralytics_ros::msg::YoloResult::SharedPtr yolo_result_msg)

    void projectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    const ultralytics_ros::YoloResultConstPtr& yolo_result_msg, const std_msgs::Header& header,
                    vision_msgs::Detection3DArray& detections3d_msg,
                    sensor_msgs::PointCloud2& combine_detection_cloud_msg); // Project point cloud onto 3D detections

    void processPointsWithBbox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                const vision_msgs::Detection2D& detection,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw); // Process points within a bounding box

    void processPointsWithMask(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const sensor_msgs::Image& mask,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw); // Process points with a mask

    pcl::PointCloud<pcl::PointXYZ>::Ptr msg2TransformedCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg); // Transform point cloud message to pcl format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                const std_msgs::Header& header); // Transform point cloud to another frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud); // Perform Euclidean clustering

    void createBoundingBox(vision_msgs::Detection3DArray& detections3d_msg,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            const std::vector<vision_msgs::ObjectHypothesisWithPose>& detections_results); // Create bounding boxes around detections
                            
    visualization_msgs::MarkerArray createMarkerArray(const vision_msgs::Detection3DArray& detections3d_msg,
                                                    const double& duration); // Create marker array for visualization
};
