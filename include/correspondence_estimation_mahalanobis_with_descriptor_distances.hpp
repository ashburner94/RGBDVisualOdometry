#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_WITH_DESCRIPTOR_DISTANCES_IMPL_HPP
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_WITH_DESCRIPTOR_DISTANCES_IMPL_HPP

#include <algorithm>
#include <opencv2/opencv.hpp>

template <typename PointSource, typename PointTarget, typename Scalar>
bool pcl::registration::
    CorrespondenceEstimationMahalanobisWithDescriptorDistances<
        PointSource, PointTarget, Scalar>::initCompute() {
  if (!CorrespondenceEstimationBase<PointSource, PointTarget,
                                    Scalar>::initCompute())
    return false;

  // Check if both clouds have the same datatype
  if (!isSamePointType<PointSource, PointTarget>()) return false;

  // Check if 0 < alpha < 1.0
  if (alpha_ < 0.0 || alpha_ > 1.0) return false;

  return true;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::
    CorrespondenceEstimationMahalanobisWithDescriptorDistances<
        PointSource, PointTarget,
        Scalar>::determineCorrespondences(pcl::Correspondences& correspondences,
                                          double max_distance) {
  // Check, if the algorithm has been initialized
  if (!initCompute()) return;

  // Init some needed variables
  correspondences.clear();
  std::vector<int> indices(num_search_points_);
  std::vector<float> distances(num_search_points_);

  // Iterate over the points
  for (int i = 0; i < (int)input_->points.size(); i++) {
    // Search for the points with the smallest euclidean distance
    tree_->nearestKSearch(input_->points[i], num_search_points_, indices,
                          distances);

    // Determine the mahalanobis-distance for each of those points
    calculateDistances(input_->points[i], indices, distances);

    // Choose the point with the smallest mahalanobis-distance and check if
    // the mahalanobis-distance is below the threshold
    int index_min = std::min_element(distances.begin(), distances.end()) -
                    distances.begin();
    if (distances[index_min] <= max_distance) {
      pcl::Correspondence corr(i, indices[index_min], distances[index_min]);
      correspondences.push_back(corr);
    }
  }

  // Increment the iteration counter
  iteration_counter_++;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::
    CorrespondenceEstimationMahalanobisWithDescriptorDistances<
        PointSource, PointTarget,
        Scalar>::determineReciprocalCorrespondences(pcl::Correspondences&
                                                        correspondences,
                                                    double max_distance) {
  // Basically leave this function empty by calling the determineCorrespondences
  // function
  determineCorrespondences(correspondences, max_distance);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::
    CorrespondenceEstimationMahalanobisWithDescriptorDistances<
        PointSource, PointTarget,
        Scalar>::calculateDistances(const PointSource& point,
                                    std::vector<int>& index,
                                    std::vector<float>& distance) {
  // Check, if the vectors have the same size
  if (index.size() == distance.size()) {
    for (int i = 0; i < (int)index.size(); i++) {
      // Calculate the distances
      cv::Mat desc_source(32, 1, CV_8UC1,
                          (unsigned char*)point.getDescriptor8U().data());
      cv::Mat desc_target(
          32, 1, CV_8UC1,
          (unsigned char*)target_->at(index[i]).getDescriptor8U().data());
      double descriptor_distance =
          cv::norm(desc_source, desc_target, cv::NORM_HAMMING);
      Eigen::Vector3f delta =
          point.getVector3fMap() - target_->at(index[i]).getVector3fMap();
      double mahalanobis_distance = std::sqrt(
          delta.transpose() *
          (point.getSigmaMatrix3f() + target_->at(index[i]).getSigmaMatrix3f())
              .inverse() *
          delta);

      // Mix the distances
      double weight = std::pow(alpha_, iteration_counter_);
      distance[i] =
          (1.0 - weight) * mahalanobis_distance + weight * descriptor_distance;
    }
  }
}

#endif /*PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_WITH_DESCRIPTOR_DISTANCES_IMPL_HPP*/
