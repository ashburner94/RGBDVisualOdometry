#include "../include/environment_model.h"
#include "../include/correspondence_estimation_mahalanobis.h"

#include <algorithm>

EnvironmentModel::EnvironmentModel(EnvironmentModelParams& params,
                                   MGFeatureCloudPtr initial_cloud) {
  // Save the parameters
  setParams(params);

  // Generate a new environment model
  environment_model.reset(new MGFeatureCloud);
  environment_model->header.frame_id = "environment_model";
  *environment_model += *initial_cloud;

  // Generate a new kdtree
  tree.reset(new pcl::search::KdTree<MGFeature, pcl::KdTreeFLANN<MGFeature>>);
  tree->setInputCloud(environment_model);
}

void EnvironmentModel::updateEnvironmentModel(
    MGFeatureCloudPtr transformed_data) {
  // Determine the correspondences
  pcl::registration::CorrespondenceEstimationMahalanobis<MGFeature, MGFeature,
                                                         float>
      correspondence_estimation;
  correspondence_estimation.setInputSource(transformed_data);
  correspondence_estimation.setInputTarget(environment_model);
  correspondence_estimation.setSearchMethodTarget(tree, true);
  pcl::Correspondences correspondences;
  correspondence_estimation.determineCorrespondences(
      correspondences, params_.max_correspondence_distance);

  // Iterate over the data
  for (int i = 0; i < (int)transformed_data->points.size(); i++) {
    // Check, if a point has a correspondence
    pcl::Correspondences::iterator correspondence_it =
        std::find_if(correspondences.begin(), correspondences.end(),
                     [i](pcl::Correspondence correspondence) {
                       return correspondence.index_query == i &&
                              correspondence.index_match != -1;
                     });

    // Decide wether to add or to update the point
    if (correspondence_it != correspondences.end()) {
      // Update the point
      // Kalman gain matrix
      Eigen::Matrix3f K = environment_model->at(correspondence_it->index_match)
                              .getSigmaMatrix3f() *
                          (environment_model->at(correspondence_it->index_match)
                               .getSigmaMatrix3f() +
                           transformed_data->at(correspondence_it->index_query)
                               .getSigmaMatrix3f())
                              .inverse();
      // Means
      environment_model->at(correspondence_it->index_match).getVector3fMap() =
          environment_model->at(correspondence_it->index_match)
              .getVector3fMap() +
          K * (transformed_data->at(correspondence_it->index_query)
                   .getVector3fMap() -
               environment_model->at(correspondence_it->index_match)
                   .getVector3fMap());
      // Covariance
      environment_model->at(correspondence_it->index_match).getSigmaMatrix3f() =
          (Eigen::Matrix3f::Identity() - K) *
          environment_model->at(correspondence_it->index_match)
              .getSigmaMatrix3f();
    } else {
      // Check if the point doesn't exceed the max variance
      bool variance_in_range = true;
      for (int j = 0; j < 3; j++) {
        if (transformed_data->at(i).getSigmaMatrix3f()(j, j) >
            params_.max_variance_position)
          variance_in_range = false;
      }
      if (variance_in_range) {
        // Add the point to the point cloud
        environment_model->push_back(transformed_data->at(i));
      }
    }
  }

  // Remove the oldest data if the maximum model size has been exceeded
  if ((int)environment_model->size() > params_.max_num_datapoints)
    environment_model->erase(
        environment_model->begin(),
        environment_model->begin() +
            (environment_model->size() - params_.max_num_datapoints));

  // Update the kdtree
  tree->setInputCloud(environment_model);
}

void EnvironmentModel::setParams(EnvironmentModelParams& params) {
  // Save the parameters
  params_ = params;
}
