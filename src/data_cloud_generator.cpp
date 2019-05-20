#include "../include/data_cloud_generator.h"

DataCloudGenerator::DataCloudGenerator(DataCloudGeneratorParams& params) {
  // Save the parameters
  setParams(params);
}

void DataCloudGenerator::generateData(MGFeatureCloudPtr& data_cloud,
                                      CameraMeasurement& measurement) {
  // Detect the features in the images
  std::vector<cv::KeyPoint> keypoints;
  cv::Ptr<cv::GFTTDetector> feature_detector = cv::GFTTDetector::create();
  feature_detector->setMaxFeatures(
      params_.feature_detector_params.max_num_features);
  feature_detector->setQualityLevel(
      params_.feature_detector_params.quality_level);
  feature_detector->setMinDistance(
      params_.feature_detector_params.min_distance);
  feature_detector->detect(measurement.greyscale, keypoints);

  // Draw the keypoints into the greyscale image
  cv::drawKeypoints(measurement.greyscale, keypoints, image_with_keypoints);

  // Calculate the descriptors of the features
  cv::Mat descriptors;
  cv::Ptr<cv::ORB> descriptor_calculator = cv::ORB::create();
  descriptor_calculator->compute(measurement.greyscale, keypoints, descriptors);

  // Empty the point cloud
  data_cloud.reset(new MGFeatureCloud);

  // Set the correct frame
  data_cloud->header.frame_id = "base_link";

  // Add a point to the data pointcloud for each feature
  for (unsigned int i = 0; i < keypoints.size(); i++) {
    // Get the correct keypoint
    cv::KeyPoint& keypoint = keypoints[i];

    // Calculate mean z, var_z
    float mean_z, var_z;
    getGaussianMixtureDistribution(measurement.depth, keypoint.pt, mean_z,
                                   var_z);
    if (std::isfinite(mean_z) && std::isfinite(var_z)) {
      // Generate a new datapoint
      MGFeature point;

      // Set the mean vector
      point.getVector3fMap()
          << (mean_z / measurement.intrinsic_params.f_x) *
                 (keypoint.pt.x - measurement.intrinsic_params.c_x),
          (mean_z / measurement.intrinsic_params.f_y) *
              (keypoint.pt.y - measurement.intrinsic_params.c_y),
          mean_z;

      // Calculate the Covariance matrix
      double var_x =
          (var_z *
               std::pow(keypoint.pt.x - measurement.intrinsic_params.c_x, 2) +
           params_.camera_params.var_u * (std::pow(mean_z, 2) + var_z)) /
          std::pow(measurement.intrinsic_params.f_x, 2);
      double var_y =
          (var_z *
               std::pow(keypoint.pt.y - measurement.intrinsic_params.c_y, 2) +
           params_.camera_params.var_v * (std::pow(mean_z, 2) + var_z)) /
          std::pow(measurement.intrinsic_params.f_y, 2);
      double var_xz = var_z *
                      (keypoint.pt.x - measurement.intrinsic_params.c_x) /
                      measurement.intrinsic_params.f_x;
      double var_yz = var_z *
                      (keypoint.pt.y - measurement.intrinsic_params.c_y) /
                      measurement.intrinsic_params.f_y;
      double var_xy =
          var_z * (keypoint.pt.x - measurement.intrinsic_params.c_x) *
          (keypoint.pt.y - measurement.intrinsic_params.c_y) /
          (measurement.intrinsic_params.f_x * measurement.intrinsic_params.f_y);

      // Set the covariance matrix
      point.getSigmaMatrix3f() << var_x, var_xy, var_xz, var_xy, var_y, var_yz,
          var_xz, var_yz, var_z;

      // Set the feature descriptor
      point.getDescriptor8U() = pcl::Descriptor8U::Map(descriptors.row(i).data);

      // Add the point to the cloud
      data_cloud->push_back(point);
    }
  }

  // Transform the data cloud into the base_links frame
  pcl::transformPointCloud(*data_cloud, *data_cloud,
                           measurement.tf_image_to_base);
  for (auto& point : *data_cloud) {
    point.getSigmaMatrix3f() =
        measurement.tf_image_to_base.rotation().cast<float>() *
        point.getSigmaMatrix3f() *
        measurement.tf_image_to_base.rotation().transpose().cast<float>();
  }
}

void DataCloudGenerator::getGaussianMixtureDistribution(cv::Mat& depth_image,
                                                        cv::Point2f feature,
                                                        float& mean_z,
                                                        float& var_z) {
  // Set the window's border
  int u_start = std::max((int)feature.x - 1, 0);
  int v_start = std::max((int)feature.y - 1, 0);
  int u_end = std::min((int)feature.x + 1, depth_image.cols - 1);
  int v_end = std::min((int)feature.y + 1, depth_image.rows - 1);

  // Init the needed variables
  mean_z = 0.0;
  var_z = 0.0;
  double alpha_sum = 0.0;
  double weight_sum = 0.0;

  // Iterate over the window
  for (int u = u_start; u <= u_end; ++u)
    for (int v = v_start; v <= v_end; ++v) {
      // Determine the weight
      double weight = 1.0;
      if (feature.x == u && feature.y == v)
        weight = 4.0;
      else if (feature.x == u || feature.y == v)
        weight = 2.0;

      // Aggregate the mean
      float z = depth_image.at<float>(v, u);
      if (std::isfinite(z) && z != 0.0) {
        // Aggregate mean and variance
        mean_z += weight * z;
        double var_z_neighbor =
            std::pow(params_.camera_params.k_z, 2) * std::pow(z, 4);
        alpha_sum += weight * (var_z_neighbor + std::pow(z, 2));

        // Sum up the weights
        weight_sum += weight;
      }
    }

  // Check, if at least one value of z was valid
  if (weight_sum != 0.0) {
    mean_z /= weight_sum;
    var_z = alpha_sum / weight_sum - std::pow(mean_z, 2);
  } else {
    mean_z = std::numeric_limits<double>::quiet_NaN();
    var_z = std::numeric_limits<double>::quiet_NaN();
  }
}

void DataCloudGenerator::setParams(DataCloudGeneratorParams& params) {
  // Save the parameters
  params_ = params;
}
