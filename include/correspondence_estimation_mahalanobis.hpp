#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_IMPL_HPP
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_IMPL_HPP

#include <algorithm>

template <typename PointSource, typename PointTarget, typename Scalar>
bool pcl::registration::CorrespondenceEstimationMahalanobis<
    PointSource, PointTarget, Scalar>::initCompute() {
  if (!CorrespondenceEstimationBase<PointSource, PointTarget,
                                    Scalar>::initCompute())
    return false;

  // Check if both clouds have the same datatype
  if (!isSamePointType<PointSource, PointTarget>()) return false;

  return true;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::CorrespondenceEstimationMahalanobis<
    PointSource, PointTarget,
    Scalar>::determineCorrespondences(pcl::Correspondences& correspondences,
                                      double max_distance) {
  // Check, if the algorithm has been initialized
  if (!initCompute()) return;

  // Init some needed variables
  correspondences.clear();
  std::vector<int> indices(num_search_points);
  std::vector<float> distances(num_search_points);

  // Iterate over the points
  for (int i = 0; i < (int)input_->points.size(); i++) {
    // Search for the points with the smallest euclidean distance
    tree_->nearestKSearch(input_->points[i], num_search_points, indices,
                          distances);

    // Determine the mahalanobis-distance for each of those points
    calculateMahalanobisDistance(input_->points[i], indices, distances);

    // Choose the point with the smallest mahalanobis-distance and check if
    // the mahalanobis-distance is below the threshold
    int index_min = std::min_element(distances.begin(), distances.end()) -
                    distances.begin();
    if (distances[index_min] <= max_distance) {
      pcl::Correspondence corr(i, indices[index_min], distances[index_min]);
      correspondences.push_back(corr);
    }
  }
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::CorrespondenceEstimationMahalanobis<
    PointSource, PointTarget,
    Scalar>::determineReciprocalCorrespondences(pcl::Correspondences&
                                                    correspondences,
                                                double max_distance) {
  // Basically leave this function empty by calling the determineCorrespondences
  // function
  determineCorrespondences(correspondences, max_distance);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::CorrespondenceEstimationMahalanobis<
    PointSource, PointTarget,
    Scalar>::calculateMahalanobisDistance(const PointSource& point,
                                          std::vector<int>& index,
                                          std::vector<float>& distance) {
  // Check, if the vectors have the same size
  if (index.size() == distance.size()) {
    for (int i = 0; i < (int)index.size(); i++) {
      Eigen::Vector3f delta =
          point.getVector3fMap() - target_->at(index[i]).getVector3fMap();
      distance[i] =
          std::sqrt(delta.transpose() *
                    (point.getSigmaMatrix3f() +
                     target_->at(index[i]).getSigmaMatrix3f()).inverse() *
                    delta);
    }
  }
}

#endif /*PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_IMPL_HPP*/
