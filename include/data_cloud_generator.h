#ifndef DATA_CLOUD_GENERATOR_H
#define DATA_CLOUD_GENERATOR_H

// include order matters at this point
// clang-format off
#include "../include/mg_feature.h"
#include <opencv2/opencv.hpp>
// clang-format on

class DataCloudGenerator {
 public:
  /**
   * @brief Parameters for feature detector.
   */
  typedef struct {
    int max_num_features; /*!< Maximum numer of features detected per frame. */
    double quality_level; /*!< Quality level of the features. */
    double min_distance;  /*!< Minimum distance between the features. */
  } FeatureDetectorParams;
  /**
   * @brief Parameters of the camera.
   */
  typedef struct {
    float var_u; /*!< Position uncertainty in u-direction */
    float var_v; /*!< Position uncertainty in v-direction */
    float k_z; /*!< Factor for calculating the uncertainty in z-direction; k_z =
                  1/(f*b) * subpixel_accuracy */
  } CameraParams;
  /**
   * @brief Wrapper for the camera's intrinsic parameters.
   */
  typedef struct {
    float f_x;  /*!< Focal length in x-direction */
    double c_x; /*!< Center of the imager in x-direction */
    float f_y;  /*!< Focal length in y-direction */
    double c_y; /*!< Center of the imager in x-direction */
  } IntrinsicParams;
  /**
   * @brief Wrapper for measurements from the camera and its position
   * information.
   */
  typedef struct {
    cv::Mat greyscale; /*!< Measurement from the greyscale sensor. */
    cv::Mat depth;     /*!< Measurement from the depth sensor. */
    Eigen::Affine3d tf_image_to_base; /*!< Transformation from the camera image
                                         to the base_link */
    IntrinsicParams
        intrinsic_params; /*!< Intrinsic parameters of the camera. */
  } CameraMeasurement;
  /**
   * @brief Parameters of the data cloud generator.
   */
  typedef struct {
    FeatureDetectorParams
        feature_detector_params; /*!< Parameters of the feature detector. */
    CameraParams camera_params;  /*!< Parameters of the camera. */
  } DataCloudGeneratorParams;
  /**
   * @brief Constructor for the data cloud generator. Sets the parameters of the
   * generator.
   * @param params Params to set.
   */
  explicit DataCloudGenerator(DataCloudGeneratorParams& params);
  /**
   * @brief Generate the data (Position of a feature + covariance) from an image
   * frame.
   * @param data_cloud Pointcloud to store the data in.
   * @param measurement Measurement from the camera.
   */
  void generateData(MGFeatureCloudPtr& data_cloud,
                    CameraMeasurement& measurement);
  /**
   * @brief Get the gaussian distribution of a feature in an image.
   * @param depth_image Depth map for the features.
   * @param feature Detected feature in the image.
   * @param mean_z Mean of the z-coordinate.
   * @param var_z Variance of the z-coordinate.
   */
  void getGaussianMixtureDistribution(cv::Mat& depth_image, cv::Point2f feature,
                                      float& mean_z, float& var_z);
  /**
   * @brief Sets new parameters for the data cloud generator.
   * @param params New parameters to set.
   */
  void setParams(DataCloudGeneratorParams& params);
  /**
   * @brief Greyscale image with the detected keypoints drawn in it.
   */
  cv::Mat image_with_keypoints;

 private:
  /**
   * @brief Parameters of the data cloud generator.
   */
  DataCloudGeneratorParams params_;
};

#endif /*DATA_CLOUD_GENERATOR_H*/
