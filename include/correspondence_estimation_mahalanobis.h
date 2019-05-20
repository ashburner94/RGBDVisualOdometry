#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_H
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_H

#include <pcl/registration/correspondence_estimation.h>
#include "mg_feature.h"

namespace pcl {
namespace registration {
/** @brief CorrespondenceEstimationMahalanobis computes correspondences by using
 * the mahalanobis distance
  * as a distance measure.
  */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class CorrespondenceEstimationMahalanobis
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

  typedef boost::shared_ptr<CorrespondenceEstimationMahalanobis<
      PointSource, PointTarget, Scalar> > Ptr;
  typedef boost::shared_ptr<const CorrespondenceEstimationMahalanobis<
      PointSource, PointTarget, Scalar> > ConstPtr;

  /**
   * @brief Empty constructor.
   */
  CorrespondenceEstimationMahalanobis() : num_search_points(4) {}

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
    Ptr copy(new CorrespondenceEstimationMahalanobis<PointSource, PointTarget,
                                                     Scalar>(*this));
    return (copy);
  }

  /**
   * @brief Determine the mahalanobis-distance between a point in the target
   * cloud and the given point.
   * @param point Given point.
   * @param index Index of the neighbour in the target cloud.
   * @param distance Distance to the neighbour in the target cloud.
   */
  void calculateMahalanobisDistance(const PointSource& point,
                                    std::vector<int>& index,
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
  unsigned int num_search_points;
};
}
}

#include "correspondence_estimation_mahalanobis.hpp"

#endif /*PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_MAHALANOBIS_H*/
