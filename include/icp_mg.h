#ifndef PCL_ICP_MG_H
#define PCL_ICP_MG_H

#include <pcl/registration/icp.h>
#include "transformation_estimation_translation_only.h"
#include "correspondence_estimation_mahalanobis.h"
#include "correspondence_estimation_mahalanobis_with_descriptor_distances.h"

namespace pcl {
/** \brief @b IterativeClosestMahalanobis is an ICP variant that uses the
 * mahalanobis-distance for
 * the correspondence estimation. For the registration a point to plane
 * algorithm is used.
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointMixedGaussian
    : public IterativeClosestPoint<PointSource, PointTarget, Scalar> {
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
  using IterativeClosestPoint<PointSource, PointTarget,
                              Scalar>::transformation_estimation_;
  using IterativeClosestPoint<PointSource, PointTarget,
                              Scalar>::computeTransformation;
  using Registration<PointSource, PointTarget,
                     Scalar>::correspondence_estimation_;
  using Registration<PointSource, PointTarget, Scalar>::correspondences_;

 public:
  typedef boost::shared_ptr<IterativeClosestPointMixedGaussian<
      PointSource, PointTarget, Scalar> > Ptr;
  typedef boost::shared_ptr<const IterativeClosestPointMixedGaussian<
      PointSource, PointTarget, Scalar> > ConstPtr;

  typedef
      typename Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;
  /**
   * @brief Constructor for IterativeClosestPointMixedGaussian. Sets the used
   * algorithms.
   */
  IterativeClosestPointMixedGaussian() {
    reg_name_ = "IterativeClosestPointMixedGaussian";

    // Set the desired correspondece estimation algorithm
    correspondence_estimation_.reset(
        new pcl::registration::
            CorrespondenceEstimationMahalanobisWithDescriptorDistances<
                PointSource, PointTarget, Scalar>);

    // Set the desired transformation estimation algorithm
    transformation_estimation_.reset(
        new pcl::registration::TransformationEstimationSVD<
            PointSource, PointTarget, Scalar>);
  }
  /**
   * @brief Get the last calculated correspondences between the input cloud and
   * the target cloud.
   * @return The last calculated correspondences.
   */
  Correspondences getCorrespondences() { return *correspondences_; }
};
}

#endif /*PCL_ICP_MG_H*/
