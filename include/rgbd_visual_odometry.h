#ifndef RGBD_VISUAL_ODOMETRY_H
#define RGBD_VISUAL_ODOMETRY_H

// Include order matters at this point.
// OpenCV leaks defines into the pcl lib.
// Shame on you OpenCV :(
#include "data_cloud_generator.h"
#include "environment_model.h"
#include "icp_mg.h"
#include "mg_feature.h"
#include "transformation_estimator.h"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class RGBDVisualOdometry {
 public:
  /**
   * @brief Params for the visual odometry.
   */
  typedef struct {
    bool use_tf_guess; /*!< True: Use external initial guess for the
                  transformation
                  between current frame and last frame. False: Use last
                  transformation estimate as current guess. */
    DataCloudGenerator::DataCloudGeneratorParams
        data_cloud_generator_params; /*!< Parameters for the data cloud
                                        generator. */
    TransformationEstimator::TransformationEstimatorParams
        transformation_estimator_params; /*!< Parameters for the
                                            transformation estimator. */
    EnvironmentModel::EnvironmentModelParams environment_model_params; /*!<
            Params for the environment model including default
            values from the paper. */
  } RGBDVisualOdometryParams;
  /**
   * @brief Constructor for the visual odometry. Sets the parameters.
   * @param params The parameters to set.
   */
  explicit RGBDVisualOdometry(RGBDVisualOdometryParams& params);
  /**
   * @brief Process an incoming rgbd frame.
   * @param measurement Measurement and position information of sensor.
   * @param external_tf_guess Initial guess of the transformation of the camera
   * base between last and current frame. Ignored if use_transformation_guess is
   * set to false.
   * @param tf_estimate Result of the transformation estimation.
   */
  void processRGBDFrame(
      DataCloudGenerator::CameraMeasurement& measurement,
      Eigen::Affine3d& external_tf_guess,
      TransformationEstimator::TransformationEstimation& tf_estimate);
  /**
   * @brief Set the parameters of the visual odometry.
   * @param params Parameters to set.
   */
  void setParams(RGBDVisualOdometryParams& params);
  /**
   * @brief Cloud containing the current measurement.
   * Note: it is not private so it can be accessed from outside for debugging.
   */
  MGFeatureCloudPtr data_cloud;
  /**
   * @brief Model containing the previous aligned measurements.
   */
  std::shared_ptr<EnvironmentModel> environment_model;
  /**
   * @brief Generator for the data clouds.
   */
  std::shared_ptr<DataCloudGenerator> data_cloud_generator;
  /**
   * @brief Estimator for the transformation between the current measurement and
   * the model.
   */
  std::shared_ptr<TransformationEstimator> transformation_estimator;

 private:
  /**
   * @brief Parameters of the visual odometry.
   */
  RGBDVisualOdometryParams params_;
  /**
   * @brief Estimate of the transformation in the last frame.
   */
  Eigen::Affine3d last_tf_estimate_;
  /**
   * @brief Inidicator whether the visual odometry has been reset.
   */
  bool is_reset;
};

#endif /*RGBD_VISUAL_ODOMETRY_H*/
