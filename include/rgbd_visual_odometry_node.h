#ifndef RGBD_VISUAL_ODOMETRY_NODE_H
#define RGBD_VISUAL_ODOMETRY_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

#include "rgbd_visual_odometry.h"

class RGBDVisualOdometryNode {
 public:
  /**
   * @brief Init all the publishers/subscribers and generate an object
   * of the visual odometry.
   */
  explicit RGBDVisualOdometryNode(ros::NodeHandle nh);

 private:
  /**
   * @brief Handle for the ROS node.
   */
  ros::NodeHandle nh_;
  /**
   * @brief Object of the visual odometry algorithm.
   */
  std::shared_ptr<RGBDVisualOdometry> visual_odometry_;
  /**
   * @brief Buffer for transforms.
   */
  tf2_ros::Buffer tf_buffer_;
  /**
   * @brief Listener for transforms.
   */
  tf2_ros::TransformListener tf_listener_;
  /**
   * @brief Subscriber for the camera image.
   */
  message_filters::Subscriber<sensor_msgs::Image> greyscale_image_sub_;
  /**
   * @brief Subscriber for the greyscale camera's intrinsics.
   */
  message_filters::Subscriber<sensor_msgs::CameraInfo>
      greyscale_camera_info_sub_;
  /**
   * @brief Subscriber for the depth image.
   */
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
  /**
   * @brief Sync policy for the approximate time synchronizer.
   */
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
      SyncPolicy;
  /**
   * @brief Time synchronizer for synchronizing messages. Puts multipile
   * synchronized messages in one callback.
   */
  message_filters::Synchronizer<SyncPolicy> time_synchronizer_;
  /**
   * @brief Broadcaster for the transform model frame to odom.
   */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  /**
   * @brief Publisher for the measurement point cloud.
   */
  ros::Publisher data_pub_;
  /**
   * @brief Publisher for the model's point cloud.
   */
  ros::Publisher model_pub_;
  /**
   * @brief Publisher for the estimated transformation.
   */
  ros::Publisher odometry_pub_;
  /**
   * @brief Publisher for the runtime of the visual odometry.
   */
  ros::Publisher runtime_pub_;
  /**
   * @brief Transport channel for the image with keypoints.
   */
  image_transport::ImageTransport image_transport_;
  /**
   * @brief Publisher for the greyscale image with keypoints.
   */
  image_transport::Publisher keypoints_image_pub_;
  /**
   * @brief Service for resetting the visual odometry.
   */
  ros::ServiceServer reset_server_;
  /**
   * @brief Timer which triggers a reload of the params.
   */
  ros::Timer get_params_timer_;
  /**
   * @brief Parameters for the visual odometry.
   */
  RGBDVisualOdometry::RGBDVisualOdometryParams params_;
  /**
   * @brief Last time the visual odometry estimated a valid pose. Used to reset
   * the visual odometry if it didn't generate a valid post for some time.
   */
  ros::Time last_fix_time;
  /**
   * @brief Timeout after which a reset is triggered.
   */
  float fix_timeout;
  /**
   * @brief Get the params for the visual odometry.
   */
  void getParams();
  /**
   * @brief Callback for the synchronized messages.
   * @param depth_image Image from the depth camera (in meters).
   * @param greyscale_image Image from a greyscale camera.
   * @param greyscale_camera_info Parameters of the greyscale camera.
   */
  void callback(const sensor_msgs::ImageConstPtr depth_image,
                const sensor_msgs::ImageConstPtr greyscale_image,
                const sensor_msgs::CameraInfoConstPtr greyscale_camera_info);
  /**
   * @brief Make a publishable pointcloud<XYZ> out of a MGFeature pointcloud.
   * @param data MGFeature pointlcoud.
   * @param stamp Timestamp of the cloud.
   * @return Publishable pointcloud.
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudToMessage(
      MGFeatureCloudPtr data, ros::Time stamp);
  /**
   * @brief Callback for the reset service.
   * @param request Empty service request.
   * @param response Empty service response.
   * @return True if the request was handled.
   */
  bool resetCallback(std_srvs::Empty::Request& request,
                     std_srvs::Empty::Response& response);
  /**
   * @brief Reset the visual odometry.
   * @return True if success.
   */
  bool reset();
  /**
   * @brief Generate a CameraMeasurement object out of two images and a camera
   * info message.
   * @param measurement Measurement object to fill with data.
   * @param depth Measurement from the depth sensor.
   * @param greyscale Measurement from the greyscale sensor.
   * @param camera_info Camera info message.
   */
  void generateCameraMeasurement(
      DataCloudGenerator::CameraMeasurement& measurement,
      sensor_msgs::ImageConstPtr depth, sensor_msgs::ImageConstPtr greyscale,
      sensor_msgs::CameraInfoConstPtr camera_info);
};

#endif /*RGBD_VISUAL_ODOMETRY_NODE_H*/
