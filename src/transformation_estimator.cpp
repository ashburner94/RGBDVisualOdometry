#include "../include/transformation_estimator.h"
#include <math.h>

TransformationEstimator::TransformationEstimator(
    TransformationEstimatorParams& params) {
  // Set the parameters.
  setParams(params);

  // Generate new correspondence rejectors
  correspondence_rejector_median_.reset(
      new pcl::registration::CorrespondenceRejectorMedianDistance());
  correspondence_rejector_one_to_one_.reset(
      new pcl::registration::CorrespondenceRejectorOneToOne());
}

void TransformationEstimator::estimateTransformation(
    Eigen::Affine3d& tf_initial_guess, TransformationEstimation& tf_estimate,
    MGFeatureCloudPtr& data_cloud,
    std::shared_ptr<EnvironmentModel>& environment_model,
    MGFeatureCloudPtr& transformed_data) {
  // Run the icp
  pcl::IterativeClosestPointMixedGaussian<MGFeature, MGFeature> icp;
  icp.setInputSource(data_cloud);
  icp.setInputTarget(environment_model->environment_model);
  icp.setMaximumIterations(params_.max_num_iterations);
  icp.addCorrespondenceRejector(correspondence_rejector_median_);
  icp.addCorrespondenceRejector(correspondence_rejector_one_to_one_);
  icp.setSearchMethodTarget(environment_model->tree, true);
  icp.align(*transformed_data, tf_initial_guess.matrix().cast<float>());

  // Check, if the icp has converged
  if (!icp.hasConverged()) {
    throw std::runtime_error("ICP did not converge.");
  }

  // Calculate the tf estimate
  tf_estimate.pose = icp.getFinalTransformation().cast<double>();

  // Save the correspondences
  pcl::Correspondences correspondences = icp.getCorrespondences();

  // Check if the transformation estimate is in range
  if ((tf_estimate.pose.translation() - tf_initial_guess.translation()).norm() >
      params_.max_translation_delta) {
    transformed_data->clear();
    throw std::runtime_error("Translation delta out of range.");
  }
  if (std::min(
          1.0,
          std::max(-1.0, std::acos(((tf_estimate.pose.rotation() *
                                     tf_initial_guess.rotation().transpose())
                                        .trace() -
                                    1.0) /
                                   2.0))) > params_.max_rotation_delta) {
    transformed_data->clear();
    throw std::runtime_error("Rotation delta out of range.");
  }

  // Estimate the covariance
  estimateICPCovariance(tf_estimate, data_cloud,
                        environment_model->environment_model, correspondences);

  // Update the variances
  for (auto point : *transformed_data)
    point.getSigmaMatrix3f() =
        tf_estimate.pose.rotation().cast<float>() * point.getSigmaMatrix3f() *
            tf_estimate.pose.rotation().transpose().cast<float>() +
        tf_estimate.covariance.cast<float>().block<3, 3>(0, 0);
}

void TransformationEstimator::estimateICPCovariance(
    TransformationEstimation& tf_estimate, MGFeatureCloudPtr& data_cloud,
    MGFeatureCloudPtr& environment_model,
    pcl::Correspondences& correspondences) {
  // Save the transformation's parameters
  double x = tf_estimate.pose.translation()(0);
  double y = tf_estimate.pose.translation()(1);
  double z = tf_estimate.pose.translation()(2);
  double roll = atan2(tf_estimate.pose.rotation()(2, 1),
                      tf_estimate.pose.rotation()(2, 2));
  double pitch = asin(-tf_estimate.pose.rotation()(2, 0));
  double yaw = atan2(tf_estimate.pose.rotation()(1, 0),
                     tf_estimate.pose.rotation()(0, 0));

  // Initialize d2J_dx2, d2J_dzdx and cov(z)
  Eigen::Matrix<double, 6, 6> d2J_dx2 = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::MatrixXd d2J_dzdx(6, 6 * correspondences.size());
  d2J_dzdx = Eigen::MatrixXd::Zero(6, 6 * correspondences.size());
  Eigen::MatrixXd cov_z(6 * correspondences.size(), 6 * correspondences.size());
  cov_z = Eigen::MatrixXd::Zero(6 * correspondences.size(),
                                6 * correspondences.size());

  for (unsigned int i = 0; i < correspondences.size(); i++) {
    // Save the coordinates of the data point
    double dataix = data_cloud->at(correspondences[i].index_query).x;
    double dataiy = data_cloud->at(correspondences[i].index_query).y;
    double dataiz = data_cloud->at(correspondences[i].index_query).z;

    // Save the coordinates of the model point
    double modelix = environment_model->at(correspondences[i].index_match).x;
    double modeliy = environment_model->at(correspondences[i].index_match).y;
    double modeliz = environment_model->at(correspondences[i].index_match).z;

    // Calculate d2J_dx2
    Eigen::Matrix<double, 6, 6> d2J_dx2_temp;
    {
      // clang-format off
        d2J_dx2_temp(0, 0) = 2;
        d2J_dx2_temp(0, 1) = 0;
        d2J_dx2_temp(0, 2) = 0;
        d2J_dx2_temp(0, 3) = 0;
        d2J_dx2_temp(0, 4) = -2*dataix*sin(pitch)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw) - 2*dataiz*cos(pitch);
        d2J_dx2_temp(0, 5) = -2*dataix*sin(yaw)*cos(pitch) + 2*dataiy*cos(pitch)*cos(yaw);
        d2J_dx2_temp(1, 0) = 0;
        d2J_dx2_temp(1, 1) = 2;
        d2J_dx2_temp(1, 2) = 0;
        d2J_dx2_temp(1, 3) = 2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll);
        d2J_dx2_temp(1, 4) = 2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll);
        d2J_dx2_temp(1, 5) = 2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll));
        d2J_dx2_temp(2, 0) = 0;
        d2J_dx2_temp(2, 1) = 0;
        d2J_dx2_temp(2, 2) = 2;
        d2J_dx2_temp(2, 3) = 2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch);
        d2J_dx2_temp(2, 4) = 2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll);
        d2J_dx2_temp(2, 5) = 2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw));
        d2J_dx2_temp(3, 0) = 0;
        d2J_dx2_temp(3, 1) = 2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll);
        d2J_dx2_temp(3, 2) = 2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch);
        d2J_dx2_temp(3, 3) = (dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - dataiz*sin(roll)*cos(pitch))*(2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch)) + (2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y) + (2*dataix*(-sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw)) + 2*dataiy*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) - 2*dataiz*cos(pitch)*cos(roll))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll))*(2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll));
        d2J_dx2_temp(3, 4) = (2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch))*(dataix*cos(pitch)*cos(roll)*cos(yaw) + dataiy*sin(yaw)*cos(pitch)*cos(roll) - dataiz*sin(pitch)*cos(roll)) + (2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll))*(dataix*sin(roll)*cos(pitch)*cos(yaw) + dataiy*sin(roll)*sin(yaw)*cos(pitch) - dataiz*sin(pitch)*sin(roll)) + (-2*dataix*sin(roll)*cos(pitch)*cos(yaw) - 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) + 2*dataiz*sin(pitch)*sin(roll))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y);
        d2J_dx2_temp(3, 5) = (dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*(2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll)) + (2*dataix*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + 2*dataiy*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch)) + (2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y);
        d2J_dx2_temp(4, 0) = -2*dataix*sin(pitch)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw) - 2*dataiz*cos(pitch);
        d2J_dx2_temp(4, 1) = 2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll);
        d2J_dx2_temp(4, 2) = 2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll);
        d2J_dx2_temp(4, 3) = (dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - dataiz*sin(roll)*cos(pitch))*(2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll)) + (dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll))*(2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll)) + (-2*dataix*sin(roll)*cos(pitch)*cos(yaw) - 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) + 2*dataiz*sin(pitch)*sin(roll))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y);
        d2J_dx2_temp(4, 4) = (-2*dataix*sin(pitch)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw) - 2*dataiz*cos(pitch))*(-dataix*sin(pitch)*cos(yaw) - dataiy*sin(pitch)*sin(yaw) - dataiz*cos(pitch)) + (-2*dataix*cos(pitch)*cos(yaw) - 2*dataiy*sin(yaw)*cos(pitch) + 2*dataiz*sin(pitch))*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x) + (-2*dataix*sin(pitch)*sin(roll)*cos(yaw) - 2*dataiy*sin(pitch)*sin(roll)*sin(yaw) - 2*dataiz*sin(roll)*cos(pitch))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y) + (-2*dataix*sin(pitch)*cos(roll)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw)*cos(roll) - 2*dataiz*cos(pitch)*cos(roll))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (dataix*sin(roll)*cos(pitch)*cos(yaw) + dataiy*sin(roll)*sin(yaw)*cos(pitch) - dataiz*sin(pitch)*sin(roll))*(2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll)) + (dataix*cos(pitch)*cos(roll)*cos(yaw) + dataiy*sin(yaw)*cos(pitch)*cos(roll) - dataiz*sin(pitch)*cos(roll))*(2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll));
        d2J_dx2_temp(4, 5) = (dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*(2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll)) + (dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll)) + (2*dataix*sin(pitch)*sin(yaw) - 2*dataiy*sin(pitch)*cos(yaw))*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x) + (-dataix*sin(yaw)*cos(pitch) + dataiy*cos(pitch)*cos(yaw))*(-2*dataix*sin(pitch)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw) - 2*dataiz*cos(pitch)) + (-2*dataix*sin(roll)*sin(yaw)*cos(pitch) + 2*dataiy*sin(roll)*cos(pitch)*cos(yaw))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y) + (-2*dataix*sin(yaw)*cos(pitch)*cos(roll) + 2*dataiy*cos(pitch)*cos(roll)*cos(yaw))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z);
        d2J_dx2_temp(5, 0) = -2*dataix*sin(yaw)*cos(pitch) + 2*dataiy*cos(pitch)*cos(yaw);
        d2J_dx2_temp(5, 1) = 2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll));
        d2J_dx2_temp(5, 2) = 2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw));
        d2J_dx2_temp(5, 3) = (2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll)) + (2*dataix*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + 2*dataiy*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - dataiz*sin(roll)*cos(pitch)) + (2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y);
        d2J_dx2_temp(5, 4) = (2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*(dataix*sin(roll)*cos(pitch)*cos(yaw) + dataiy*sin(roll)*sin(yaw)*cos(pitch) - dataiz*sin(pitch)*sin(roll)) + (2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(dataix*cos(pitch)*cos(roll)*cos(yaw) + dataiy*sin(yaw)*cos(pitch)*cos(roll) - dataiz*sin(pitch)*cos(roll)) + (2*dataix*sin(pitch)*sin(yaw) - 2*dataiy*sin(pitch)*cos(yaw))*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x) + (-2*dataix*sin(yaw)*cos(pitch) + 2*dataiy*cos(pitch)*cos(yaw))*(-dataix*sin(pitch)*cos(yaw) - dataiy*sin(pitch)*sin(yaw) - dataiz*cos(pitch)) + (-2*dataix*sin(roll)*sin(yaw)*cos(pitch) + 2*dataiy*sin(roll)*cos(pitch)*cos(yaw))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y) + (-2*dataix*sin(yaw)*cos(pitch)*cos(roll) + 2*dataiy*cos(pitch)*cos(roll)*cos(yaw))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z);
        d2J_dx2_temp(5, 5) = (dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*(2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))) + (2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y) + (dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))) + (2*dataix*(-sin(pitch)*cos(roll)*cos(yaw) - sin(roll)*sin(yaw)) + 2*dataiy*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (-2*dataix*sin(yaw)*cos(pitch) + 2*dataiy*cos(pitch)*cos(yaw))*(-dataix*sin(yaw)*cos(pitch) + dataiy*cos(pitch)*cos(yaw)) + (-2*dataix*cos(pitch)*cos(yaw) - 2*dataiy*sin(yaw)*cos(pitch))*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x);
      // clang-format on
    }
    d2J_dx2 += d2J_dx2_temp;

    // Calculate d2J_dzdx
    Eigen::Matrix<double, 6, 6> d2J_dzdx_temp;
    {
      // clang-format off
        d2J_dzdx_temp(0, 0) = 2*cos(pitch)*cos(yaw);
        d2J_dzdx_temp(0, 1) = 2*sin(yaw)*cos(pitch);
        d2J_dzdx_temp(0, 2) = -2*sin(pitch);
        d2J_dzdx_temp(0, 3) = -2;
        d2J_dzdx_temp(0, 4) = 0;
        d2J_dzdx_temp(0, 5) = 0;
        d2J_dzdx_temp(1, 0) = 2*sin(pitch)*sin(roll)*cos(yaw) - 2*sin(yaw)*cos(roll);
        d2J_dzdx_temp(1, 1) = 2*sin(pitch)*sin(roll)*sin(yaw) + 2*cos(roll)*cos(yaw);
        d2J_dzdx_temp(1, 2) = 2*sin(roll)*cos(pitch);
        d2J_dzdx_temp(1, 3) = 0;
        d2J_dzdx_temp(1, 4) = -2;
        d2J_dzdx_temp(1, 5) = 0;
        d2J_dzdx_temp(2, 0) = 2*sin(pitch)*cos(roll)*cos(yaw) + 2*sin(roll)*sin(yaw);
        d2J_dzdx_temp(2, 1) = 2*sin(pitch)*sin(yaw)*cos(roll) - 2*sin(roll)*cos(yaw);
        d2J_dzdx_temp(2, 2) = 2*cos(pitch)*cos(roll);
        d2J_dzdx_temp(2, 3) = 0;
        d2J_dzdx_temp(2, 4) = 0;
        d2J_dzdx_temp(2, 5) = -2;
        d2J_dzdx_temp(3, 0) = (-2*sin(pitch)*sin(roll)*cos(yaw) + 2*sin(yaw)*cos(roll))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*(2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll)) + (sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*(2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch)) + (2*sin(pitch)*cos(roll)*cos(yaw) + 2*sin(roll)*sin(yaw))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y);
        d2J_dzdx_temp(3, 1) = (-2*sin(pitch)*sin(roll)*sin(yaw) - 2*cos(roll)*cos(yaw))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + (sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*(2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll)) + (sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*(2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch)) + (2*sin(pitch)*sin(yaw)*cos(roll) - 2*sin(roll)*cos(yaw))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y);
        d2J_dzdx_temp(3, 2) = (2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) + 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiz*sin(roll)*cos(pitch))*cos(pitch)*cos(roll) + (2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + 2*dataiz*cos(pitch)*cos(roll))*sin(roll)*cos(pitch) + 2*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y)*cos(pitch)*cos(roll) - 2*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z)*sin(roll)*cos(pitch);
        d2J_dzdx_temp(3, 3) = 0;
        d2J_dzdx_temp(3, 4) = -2*dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) - 2*dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) - 2*dataiz*cos(pitch)*cos(roll);
        d2J_dzdx_temp(3, 5) = -2*dataix*(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll)) - 2*dataiy*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiz*sin(roll)*cos(pitch);
        d2J_dzdx_temp(4, 0) = (sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*(2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll)) + (sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*(2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll)) + (-2*dataix*sin(pitch)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw) - 2*dataiz*cos(pitch))*cos(pitch)*cos(yaw) + 2*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y)*sin(roll)*cos(pitch)*cos(yaw) + 2*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z)*cos(pitch)*cos(roll)*cos(yaw) - 2*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x)*sin(pitch)*cos(yaw);
        d2J_dzdx_temp(4, 1) = (sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*(2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll)) + (sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*(2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll)) + (-2*dataix*sin(pitch)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw) - 2*dataiz*cos(pitch))*sin(yaw)*cos(pitch) + 2*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y)*sin(roll)*sin(yaw)*cos(pitch) + 2*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z)*sin(yaw)*cos(pitch)*cos(roll) - 2*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x)*sin(pitch)*sin(yaw);
        d2J_dzdx_temp(4, 2) = -(-2*dataix*sin(pitch)*cos(yaw) - 2*dataiy*sin(pitch)*sin(yaw) - 2*dataiz*cos(pitch))*sin(pitch) + (2*dataix*sin(roll)*cos(pitch)*cos(yaw) + 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) - 2*dataiz*sin(pitch)*sin(roll))*sin(roll)*cos(pitch) + (2*dataix*cos(pitch)*cos(roll)*cos(yaw) + 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) - 2*dataiz*sin(pitch)*cos(roll))*cos(pitch)*cos(roll) - 2*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y)*sin(pitch)*sin(roll) - 2*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z)*sin(pitch)*cos(roll) - 2*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x)*cos(pitch);
        d2J_dzdx_temp(4, 3) = 2*dataix*sin(pitch)*cos(yaw) + 2*dataiy*sin(pitch)*sin(yaw) + 2*dataiz*cos(pitch);
        d2J_dzdx_temp(4, 4) = -2*dataix*sin(roll)*cos(pitch)*cos(yaw) - 2*dataiy*sin(roll)*sin(yaw)*cos(pitch) + 2*dataiz*sin(pitch)*sin(roll);
        d2J_dzdx_temp(4, 5) = -2*dataix*cos(pitch)*cos(roll)*cos(yaw) - 2*dataiy*sin(yaw)*cos(pitch)*cos(roll) + 2*dataiz*sin(pitch)*cos(roll);
        d2J_dzdx_temp(5, 0) = (2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + (2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + (-2*dataix*sin(yaw)*cos(pitch) + 2*dataiy*cos(pitch)*cos(yaw))*cos(pitch)*cos(yaw) + (-2*sin(pitch)*sin(roll)*sin(yaw) - 2*cos(roll)*cos(yaw))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y) + (-2*sin(pitch)*sin(yaw)*cos(roll) + 2*sin(roll)*cos(yaw))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) - 2*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x)*sin(yaw)*cos(pitch);
        d2J_dzdx_temp(5, 1) = (2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + (2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + (-2*dataix*sin(yaw)*cos(pitch) + 2*dataiy*cos(pitch)*cos(yaw))*sin(yaw)*cos(pitch) + (2*sin(pitch)*sin(roll)*cos(yaw) - 2*sin(yaw)*cos(roll))*(dataix*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)) + dataiy*(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw)) + dataiz*sin(roll)*cos(pitch) - modeliy + y) + (2*sin(pitch)*cos(roll)*cos(yaw) + 2*sin(roll)*sin(yaw))*(dataix*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + dataiy*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + dataiz*cos(pitch)*cos(roll) - modeliz + z) + 2*(dataix*cos(pitch)*cos(yaw) + dataiy*sin(yaw)*cos(pitch) - dataiz*sin(pitch) - modelix + x)*cos(pitch)*cos(yaw);
        d2J_dzdx_temp(5, 2) = (2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll)))*sin(roll)*cos(pitch) + (2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) + 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)))*cos(pitch)*cos(roll) - (-2*dataix*sin(yaw)*cos(pitch) + 2*dataiy*cos(pitch)*cos(yaw))*sin(pitch);
        d2J_dzdx_temp(5, 3) = 2*dataix*sin(yaw)*cos(pitch) - 2*dataiy*cos(pitch)*cos(yaw);
        d2J_dzdx_temp(5, 4) = -2*dataix*(-sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw)) - 2*dataiy*(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll));
        d2J_dzdx_temp(5, 5) = -2*dataix*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw)) - 2*dataiy*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw));
      // clang-format on
    }
    d2J_dzdx.block<6, 6>(0, 6 * i) = d2J_dzdx_temp;

    // Calculate cov(z)
    cov_z.block<3, 3>(3 * i, 3 * i) =
        data_cloud->at(correspondences[i].index_query)
            .getSigmaMatrix3f()
            .cast<double>();
    cov_z.block<3, 3>(3 * (i + 1), 3 * (i + 1)) =
        environment_model->at(correspondences[i].index_match)
            .getSigmaMatrix3f()
            .cast<double>();
  }

  // Calculate the covariance
  tf_estimate.covariance = d2J_dx2.inverse() * d2J_dzdx * cov_z *
                           d2J_dzdx.transpose() * d2J_dx2.inverse().transpose();
}

void TransformationEstimator::setParams(TransformationEstimatorParams& params) {
  params_ = params;
}
