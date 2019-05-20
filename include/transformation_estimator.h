#ifndef TRANSFORMATION_ESTIMATOR_H
#define TRANSFORMATION_ESTIMATOR_H

// These functions are already precompiled. No need for recompilation.
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include "environment_model.h"
#include "mg_feature.h"

class TransformationEstimator {
 public:
  /**
   * @brief Wrapper for the result of the visual odometry algorithm.
   */
  typedef struct {
    Eigen::Affine3d pose; /*!< Estimate of the current pose */
    Eigen::Matrix<double, 6, 6>
        covariance; /*!< Covariance of the pose estimate */
  } TransformationEstimation;
  /**
   * @brief Parameters for the iterative closest points algorithm.
   */
  typedef struct {
    int max_num_iterations;       /*!< Maximum number of ICP iterations. */
    double max_translation_delta; /*!< Maximum translation between the
                                            guess and the current estimate */
    double max_rotation_delta;    /*!< Maximum rotation delta between the
                                            guess and the current estimate */
  } TransformationEstimatorParams;
  /**
   * @brief Constructor for the transformation estimator. Sets the parameters
   * and activates the correspondence estimators.
   * @param params New parameters to set.
   */
  explicit TransformationEstimator(TransformationEstimatorParams& params);
  /**
   * @brief Estimate the transformation between the current frame and the last
   * frame.
   * @param tf_initial_guess Initial transformation guess for the icp algorithm.
   * @param tf_estimate Transformation estimate to calculate.
   * @param data_cloud Cloud containing the current measurements.
   * @param environment_model Environment model containing the model cloud.
   * @param transformed_data Transformed data cloud.
   */
  void estimateTransformation(
      Eigen::Affine3d& tf_initial_guess, TransformationEstimation& tf_estimate,
      MGFeatureCloudPtr& data_cloud,
      std::shared_ptr<EnvironmentModel>& environment_model,
      MGFeatureCloudPtr& transformed_data);
  /**
   * @brief Set the parameters of the Transoformation Estimator.
   * @param params New parameters to set.
   */
  void setParams(TransformationEstimatorParams& params);

 private:
  /**
   * @brief Parameters of the transformation estimator.
   */
  TransformationEstimatorParams params_;
  /**
   * @brief Estimate the covariance of the icp's pose estimate.
   * @param tf_estimate Estimate of the transformation between the model and the
   * current measurement.
   * @param transformed_data Transformed measurement.
   * @param environment_model Model cloud.
   * @param correspondences Corresponding points between the model and the
   * measurement.
   */
  void estimateICPCovariance(TransformationEstimation& tf_estimate,
                             MGFeatureCloudPtr& data_cloud,
                             MGFeatureCloudPtr& environment_model,
                             pcl::Correspondences& correspondences);
  /**
   * @brief Correspondence rejector for the distances. Discards all the values
   * above the median.
   */
  pcl::registration::CorrespondenceRejectorMedianDistance::Ptr
      correspondence_rejector_median_;
  /**
   * @brief Correspondence rejector for duplicate correspondences. Checks, if a
   * target point has more than one corresponding point.
   */
  pcl::registration::CorrespondenceRejectorOneToOne::Ptr
      correspondence_rejector_one_to_one_;
};

#endif /*TRANSFORMATION_ESTIMATOR_H*/
