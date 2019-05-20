#include "../include/rgbd_visual_odometry.h"

#include <opencv2/features2d/features2d.hpp>
#include <vector>

RGBDVisualOdometry::RGBDVisualOdometry(RGBDVisualOdometryParams& params)
    : is_reset(true) {
  // Save the params
  setParams(params);

  // Generate a new data cloud generator
  data_cloud_generator.reset(
      new DataCloudGenerator(params_.data_cloud_generator_params));

  // Generate a new transformation estimator
  transformation_estimator.reset(
      new TransformationEstimator(params_.transformation_estimator_params));
}

void RGBDVisualOdometry::processRGBDFrame(
    DataCloudGenerator::CameraMeasurement& measurement,
    Eigen::Affine3d& external_tf_guess,
    TransformationEstimator::TransformationEstimation& tf_estimate) {
  // Initialise the tf_estimate
  tf_estimate.pose.setIdentity();
  tf_estimate.covariance *= std::numeric_limits<double>::infinity();

  // Generate the data
  data_cloud_generator->generateData(data_cloud, measurement);

  // Check if the visual odometry has been reset
  if (is_reset) {
    environment_model.reset(
        new EnvironmentModel(params_.environment_model_params, data_cloud));
    last_tf_estimate_ = Eigen::Affine3d::Identity();
    is_reset = false;
  }

  // Estimate the transformation
  Eigen::Affine3d tf_initial_guess;
  if (params_.use_tf_guess)
    tf_initial_guess = external_tf_guess;
  else
    tf_initial_guess = last_tf_estimate_;
  MGFeatureCloudPtr transformed_data_cloud(new MGFeatureCloud);
  transformation_estimator->estimateTransformation(
      tf_initial_guess, tf_estimate, data_cloud, environment_model,
      transformed_data_cloud);

  // Update the environment model
  environment_model->updateEnvironmentModel(transformed_data_cloud);

  // Safe the current transformation estimate
  last_tf_estimate_ = tf_estimate.pose;
}

void RGBDVisualOdometry::setParams(RGBDVisualOdometryParams& params) {
  params_ = params;
  if (environment_model != nullptr)
    environment_model->setParams(params_.environment_model_params);
  if (data_cloud_generator != nullptr)
    data_cloud_generator->setParams(params_.data_cloud_generator_params);
  if (transformation_estimator != nullptr)
    transformation_estimator->setParams(
        params_.transformation_estimator_params);
}
