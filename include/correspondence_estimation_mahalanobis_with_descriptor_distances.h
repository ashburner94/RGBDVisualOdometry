#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_WITH_DESCRIPTOR_DISTANCES_H
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_WITH_DESCRIPTOR_DISTANCES_H

#include <pcl/registration/correspondence_estimation.h>
#include "mg_feature.h"

namespace pcl {
namespace registration {
/** @brief CorrespondenceEstimationMahalanobisWithDescriptorDistances computes
 * correspondences by using
 * the mahalanobis distance
 * as a distance measure.
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class CorrespondenceEstimationMahalanobisWithDescriptorDistances
    : public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> {
 public:
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::initCompute;
  using PCLBase<PointSource>::input_;
  using PCLBase<PointSource>::indices_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::getClassName;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;

  typedef pcl::PointCloud<PointSource> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef pcl::PointCloud<PointTarget> PointCloudTarget;
  typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
  typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

  typedef boost::shared_ptr<
      CorrespondenceEstimationMahalanobisWithDescriptorDistances<
          PointSource, PointTarget, Scalar> >
      Ptr;
  typedef boost::shared_ptr<
      const CorrespondenceEstimationMahalanobisWithDescriptorDistances<
          PointSource, PointTarget, Scalar> >
      ConstPtr;
  /**
   * @brief Empty constructor.
   */
  CorrespondenceEstimationMahalanobisWithDescriptorDistances()
      : num_search_points_(4), iteration_counter_(1), alpha_(0.5) {}
  /** @brief Computes the correspondences, applying a maximum mahalanobis
   * distance threshold.
   * @param correspondences Correspondence object.
   * @param[in] max_distance Mahalanobis distance threshold above which
   * correspondences will be rejected.
   */
  void determineCorrespondences(Correspondences& correspondences,
                                double max_distance);
  /** @brief Computes the correspondences, applying a maximum Euclidean distance
   * threshold.
   * @param correspondences Correspondences object.
   * @param[in] max_distance Euclidean distance threshold above which
   * correspondences will be rejected
   */
  void determineReciprocalCorrespondences(Correspondences& correspondences,
                                          double max_distance);
  /** \brief Clone and cast to CorrespondenceEstimationBase */
  virtual boost::shared_ptr<
      CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> >
  clone() const {
    Ptr copy(new CorrespondenceEstimationMahalanobisWithDescriptorDistances<
             PointSource, PointTarget, Scalar>(*this));
    return (copy);
  }
  /**
   * @brief Determine the mahalanobis-distance between a point in the target
   * cloud and the given point and the distance of their descriptors.
   * @param point Given point.
   * @param index Index of the neighbour in the target cloud.
   * @param distance Distance to the neighbour in the target cloud.
   */
  void calculateDistances(const PointSource& point, std::vector<int>& index,
                          std::vector<float>& distance);

 protected:
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
  /**
   * @brief Init the algorithm.
   * @return True if success.
   */
  bool initCompute();
  /**
   * @brief Number of points for the search in the kd-tree.
   */
  unsigned int num_search_points_;
  /**
   * @brief Counter for the number of iterations.
   */
  unsigned int iteration_counter_;
  /**
   * @brief Parameter for calculating the weight of the mahalanobis
   * distance and the descriptor distance.
   */
  double alpha_;
};
}  // namespace registration
}  // namespace pcl

#include "correspondence_estimation_mahalanobis_with_descriptor_distances.hpp"

#endif /*PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_WITH_DESCRIPTOR_DISTANCES_H*/
