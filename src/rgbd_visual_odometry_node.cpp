#include "../include/rgbd_visual_odometry_node.h"
#include "../include/get_param.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <tf2_eigen/tf2_eigen.h>

RGBDVisualOdometryNode::RGBDVisualOdometryNode(ros::NodeHandle nh)
    : tf_listener_(tf_buffer_),
      greyscale_image_sub_(nh, "greyscale_image_rect", 10),
      greyscale_camera_info_sub_(nh, "greyscale_camera_info", 10),
      depth_image_sub_(nh, "depth_image_rect", 10),
      time_synchronizer_(SyncPolicy(10), depth_image_sub_, greyscale_image_sub_,
                         greyscale_camera_info_sub_),
      image_transport_(nh),
      last_fix_time(ros::Time::now()) {
  // Save the nodehandle
  nh_ = nh;

  // Get the params
  getParams();

  // Generate a new visual odometry object
  visual_odometry_.reset(new RGBDVisualOdometry(params_));

  // Activate the publishers
  data_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("data_cloud", 10);
  model_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("model_cloud", 10);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 10);
  runtime_pub_ = nh_.advertise<std_msgs::Float64>("runtime", 10);
  keypoints_image_pub_ = image_transport_.advertise("keypoints", 1);

  // Activate the reset service
  reset_server_ = nh_.advertiseService(
      "reset", &RGBDVisualOdometryNode::resetCallback, this);

  // Activate the parameter getter timer
  get_params_timer_ =
      nh_.createTimer(ros::Duration(2.0), [this](const ros::TimerEvent&) {
        // Get and set the parameters of the visual odometry
        getParams();
        visual_odometry_->setParams(params_);
      });

  // Bind the callback function
  time_synchronizer_.registerCallback(
      bind(&RGBDVisualOdometryNode::callback, this, _1, _2, _3));

  // Let the node run
  ros::spin();
}

void RGBDVisualOdometryNode::getParams() {
  // Visual odometry parameters
  getParam(nh_, "use_tf_guess", params_.use_tf_guess);
  getParam(nh_, "feature_detector_params/max_num_features",
           params_.data_cloud_generator_params.feature_detector_params
               .max_num_features);
  getParam(nh_, "feature_detector_params/quality_level",
           params_.data_cloud_generator_params.feature_detector_params
               .quality_level);
  getParam(
      nh_, "feature_detector_params/min_distance",
      params_.data_cloud_generator_params.feature_detector_params.min_distance);
  getParam(nh_, "camera_params/var_u",
           params_.data_cloud_generator_params.camera_params.var_u);
  getParam(nh_, "camera_params/var_v",
           params_.data_cloud_generator_params.camera_params.var_v);
  getParam(nh_, "camera_params/k_z",
           params_.data_cloud_generator_params.camera_params.k_z);
  getParam(nh_, "transformation_estimator_params/max_num_iterations",
           params_.transformation_estimator_params.max_num_iterations);
  getParam(nh_, "transformation_estimator_params/max_translation_delta",
           params_.transformation_estimator_params.max_translation_delta);
  getParam(nh_, "transformation_estimator_params/max_rotation_delta",
           params_.transformation_estimator_params.max_rotation_delta);
  getParam(nh_, "environment_model_params/max_num_datapoints",
           params_.environment_model_params.max_num_datapoints);
  getParam(nh_, "environment_model_params/max_correspondence_distance",
           params_.environment_model_params.max_correspondence_distance);
  getParam(nh_, "environment_model_params/max_variance_position",
           params_.environment_model_params.max_variance_position);
  // Node parameters
  getParam(nh_, "fix_timeout", fix_timeout);
}

void RGBDVisualOdometryNode::callback(
    const sensor_msgs::ImageConstPtr depth_image,
    const sensor_msgs::ImageConstPtr greyscale_image,
    const sensor_msgs::CameraInfoConstPtr greyscale_camera_info) {
  // Publish the model cloud's frame
  if (static_tf_broadcaster_ == nullptr) {
    try {
      geometry_msgs::TransformStamped tf_model_to_odom =
          tf_buffer_.lookupTransform("odom", "base_link",
                                     depth_image->header.stamp,
                                     ros::Duration(0.1));
      tf_model_to_odom.child_frame_id = "environment_model";
      static_tf_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster);
      static_tf_broadcaster_->sendTransform(tf_model_to_odom);
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
  }

  // Get the measurements from the image sensors
  DataCloudGenerator::CameraMeasurement measurement;
  generateCameraMeasurement(measurement, depth_image, greyscale_image,
                            greyscale_camera_info);

  // Get the current transformation guess for the icp algorithm
  Eigen::Affine3d tf_guess;
  try {
    tf_guess = tf2::transformToEigen(tf_buffer_.lookupTransform(
        "environment_model", "base_link", depth_image->header.stamp,
        ros::Duration(0.1)));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Init the runtime measurement
  ros::WallTime visual_odometry_start_time = ros::WallTime::now();
  ros::WallDuration visual_odometry_runtime;

  // Call the visual odometry
  TransformationEstimator::TransformationEstimation tf_estimate;
  try {
    visual_odometry_->processRGBDFrame(measurement, tf_guess, tf_estimate);
  } catch (std::runtime_error& ex) {
    ROS_WARN("%s", ex.what());
    // Check if the visual odometry hasn't timed out
    if (depth_image->header.stamp - last_fix_time >
        ros::Duration(fix_timeout)) {
      // Reset the visual odometry and the timer
      reset();
      last_fix_time = depth_image->header.stamp;
    }
    return;
  }

  // Calculate the runtime
  visual_odometry_runtime = ros::WallTime::now() - visual_odometry_start_time;

  // Set the last time the visual odometry generated a valid pose
  last_fix_time = depth_image->header.stamp;

  // Publish the data pointcloud
  data_pub_.publish(pointCloudToMessage(visual_odometry_->data_cloud,
                                        depth_image->header.stamp));

  // Publish the model
  model_pub_.publish(pointCloudToMessage(
      visual_odometry_->environment_model->environment_model,
      depth_image->header.stamp));

  // Publish the odometry
  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.stamp = depth_image->header.stamp;
  odometry_msg.header.frame_id = "environment_model";
  odometry_msg.child_frame_id = "base_link";
  odometry_msg.pose.pose = tf2::toMsg(tf_estimate.pose);
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covariance_buffer =
      tf_estimate.covariance;
  for (unsigned int i = 0; i < odometry_msg.pose.covariance.size(); i++)
    odometry_msg.pose.covariance[i] = covariance_buffer(i);
  odometry_pub_.publish(odometry_msg);

  // Publish the runtime
  std_msgs::Float64 runtime_msg;
  runtime_msg.data = visual_odometry_runtime.toSec();
  runtime_pub_.publish(runtime_msg);

  // Publish the keypoints image
  sensor_msgs::ImagePtr keypoints_msg =
      cv_bridge::CvImage(
          std_msgs::Header(), "bgr8",
          visual_odometry_->data_cloud_generator->image_with_keypoints)
          .toImageMsg();
  keypoints_msg->header.frame_id = greyscale_image->header.frame_id;
  keypoints_image_pub_.publish(keypoints_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDVisualOdometryNode::pointCloudToMessage(
    MGFeatureCloudPtr data, ros::Time stamp) {
  // Empty the message
  pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

  // Set the correct header data
  msg->header.frame_id = data->header.frame_id;
  msg->header.stamp = pcl_conversions::toPCL(stamp);

  // Iterate over data's points and add them to the message
  for (auto data_point : *data) {
    pcl::PointXYZ msg_point;
    msg_point.getVector4fMap() = data_point.getVector4fMap();
    msg->push_back(msg_point);
  }
  return msg;
}

bool RGBDVisualOdometryNode::resetCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {
  return reset();
}

bool RGBDVisualOdometryNode::reset() {
  // Reset the visual odometry
  getParams();
  visual_odometry_.reset(new RGBDVisualOdometry(params_));

  // Reset the visual odometry's reference frame
  static_tf_broadcaster_.reset();

  // Publish a warning
  ROS_WARN("RGBDVisualOdometry was reset!");

  return true;
}

void RGBDVisualOdometryNode::generateCameraMeasurement(
    DataCloudGenerator::CameraMeasurement& measurement,
    sensor_msgs::ImageConstPtr depth, sensor_msgs::ImageConstPtr greyscale,
    sensor_msgs::CameraInfoConstPtr camera_info) {
  measurement.greyscale = cv_bridge::toCvCopy(*greyscale)->image;
  measurement.depth = cv_bridge::toCvCopy(*depth)->image;
  try {
    measurement.tf_image_to_base =
        tf2::transformToEigen(tf_buffer_.lookupTransform(
            "base_link", depth->header.frame_id, ros::Time(0)));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  measurement.intrinsic_params.f_x = camera_info->K[0];
  measurement.intrinsic_params.c_x = camera_info->K[2];
  measurement.intrinsic_params.f_y = camera_info->K[4];
  measurement.intrinsic_params.c_y = camera_info->K[5];
}

/**
 * @brief Main Entry point of the node.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "rgbd_visual_odometry_node");
  ros::NodeHandle nh("~");
  RGBDVisualOdometryNode visual_odometry(nh);
  return 0;
}
