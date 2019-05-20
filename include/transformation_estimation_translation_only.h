#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_ONLY_H_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_ONLY_H_

#include <pcl/registration/transformation_estimation.h>
#include <pcl/cloud_iterator.h>

namespace pcl {
namespace registration {
/**
 * @brief TransformationEstimationTranslationOnly implements a estimation of
 * the translation between two point clouds. It is basically the SVD estimation
 * without the SVD. This algorithm depends on already properly rotated
 * pointclouds.
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationEstimationTranslationOnly
    : public TransformationEstimation<PointSource, PointTarget, Scalar> {
 public:
  typedef boost::shared_ptr<TransformationEstimationTranslationOnly<
      PointSource, PointTarget, Scalar> > Ptr;
  typedef boost::shared_ptr<const TransformationEstimationTranslationOnly<
      PointSource, PointTarget, Scalar> > ConstPtr;
  typedef typename TransformationEstimation<PointSource, PointTarget,
                                            Scalar>::Matrix4 Matrix4;
  /**
   * @brief Empty constructor.
   */
  TransformationEstimationTranslationOnly() {}
  /**
   * @brief Empty destructor.
   */
  virtual ~TransformationEstimationTranslationOnly(){};
  /**
   * @brief Estimate the translation between two given point clouds.
   * @param cloud_src The source point cloud dataset.
   * @param[in] cloud_tgt The target point cloud dataset.
   * @param[out] transformation_matrix The resultant transformation matrix.
   */
  inline void estimateRigidTransformation(
      const pcl::PointCloud<PointSource> &cloud_src,
      const pcl::PointCloud<PointTarget> &cloud_tgt,
      Matrix4 &transformation_matrix) const;

  /**
   * @brief Estimate the translation between two given point clouds.
   * @param cloud_src The source point cloud dataset.
   * @param indices_src The vector of indices describing the points of
   * interest in \a cloud_src.
   * @param cloud_tgt The target point cloud dataset.
   * @param transformation_matrix The resultant transformation matrix
   */
  inline void estimateRigidTransformation(
      const pcl::PointCloud<PointSource> &cloud_src,
      const std::vector<int> &indices_src,
      const pcl::PointCloud<PointTarget> &cloud_tgt,
      Matrix4 &transformation_matrix) const;

  /**
   * @brief Estimate the translation between two given point clouds.
   * @param cloud_src The source point cloud dataset.
   * @param indices_src The vector of indices describing the points of
   * interest in \a cloud_src.
   * @param cloud_tgt The target point cloud dataset.
   * @param indices_tgt The vector of indices describing the
   * correspondences of the interest points from \a indices_src.
   * @param transformation_matrix The resultant transformation matrix.
   */
  inline void estimateRigidTransformation(
      const pcl::PointCloud<PointSource> &cloud_src,
      const std::vector<int> &indices_src,
      const pcl::PointCloud<PointTarget> &cloud_tgt,
      const std::vector<int> &indices_tgt,
      Matrix4 &transformation_matrix) const;

  /**
   * @brief Estimate the translation between two given point clouds.
   * @param cloud_src The source point cloud dataset.
   * @param cloud_tgt The target point cloud dataset.
   * @param correspondences The vector of correspondences between source
   * and target point cloud.
   * @param transformation_matrix The resultant transformation matrix.
   */
  void estimateRigidTransformation(
      const pcl::PointCloud<PointSource> &cloud_src,
      const pcl::PointCloud<PointTarget> &cloud_tgt,
      const pcl::Correspondences &correspondences,
      Matrix4 &transformation_matrix) const;

 protected:
  /**
   * @brief Estimate the translation between two given point clouds.
   * @param source_it An iterator over the source point cloud dataset
   * @param target_it An iterator over the target point cloud dataset
   * @param transformation_matrix The resultant transformation matrix
   */
  void estimateRigidTransformation(ConstCloudIterator<PointSource> &source_it,
                                   ConstCloudIterator<PointTarget> &target_it,
                                   Matrix4 &transformation_matrix) const;
};
}
}

#include "transformation_estimation_translation_only.hpp"

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_ONLY_H_ */
