#ifndef ENVIRONMENT_MODEL_H
#define ENVIRONMENT_MODEL_H

#include <pcl/search/kdtree.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../include/icp_mg.h"
#include "../include/mg_feature.h"

class EnvironmentModel {
 public:
  /**
   * @brief Params for the environment model.
   */
  typedef struct {
    int max_num_datapoints; /*!< Maxmimum number of points in the model. */
    double max_correspondence_distance; /*!< Maximum distance between two
                                           corresponding points. */
    double max_variance_position;       /*!< Maximum variance of a datapoints
                                           poisiton. */
  } EnvironmentModelParams;
  /**
   * @brief Construct a new environment model and set its parameters.
   * @param params Parameters of the cloud.
   * @param initial_cloud Initial feature cloud to build to model on.
   */
  EnvironmentModel(EnvironmentModelParams& params,
                   MGFeatureCloudPtr initial_cloud);
  /**
   * @brief Point cloud of the detected features.
   */
  MGFeatureCloudPtr environment_model;
  /**
   * @brief Update the environment model by adding/removing points.
   * @param transformed_data Transformed measurement in the reference frame.
   */
  void updateEnvironmentModel(MGFeatureCloudPtr transformed_data);
  /**
   * @brief Set the parameters of the environment model.
   * @param params Parameters to set.
   */
  void setParams(EnvironmentModelParams& params);
  /**
   * @brief KdTree of the model.
   */
  pcl::search::KdTree<MGFeature, pcl::KdTreeFLANN<MGFeature>>::Ptr tree;

 private:
  /**
   * @brief Params of the environment model.
   */
  EnvironmentModelParams params_;
};

#endif /*ENVIRONMENT_MODEL_H*/
