#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_ONLY_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_ONLY_HPP_
#include <pcl/common/eigen.h>

template <typename PointSource, typename PointTarget, typename Scalar>
inline void pcl::registration::TransformationEstimationTranslationOnly<
    PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource> &cloud_src,
                                const pcl::PointCloud<PointTarget> &cloud_tgt,
                                Matrix4 &transformation_matrix) const {
  size_t nr_points = cloud_src.points.size();
  if (cloud_tgt.points.size() != nr_points) {
    PCL_ERROR(
        "[pcl::TransformationEstimationTranslationOnly::"
        "estimateRigidTransformation] Number or points in source (%lu) differs "
        "than target (%lu)!\n",
        nr_points, cloud_tgt.points.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::TransformationEstimationTranslationOnly<
    PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource> &cloud_src,
                                const std::vector<int> &indices_src,
                                const pcl::PointCloud<PointTarget> &cloud_tgt,
                                Matrix4 &transformation_matrix) const {
  if (indices_src.size() != cloud_tgt.points.size()) {
    PCL_ERROR(
        "[pcl::TransformationSVD::estimateRigidTransformation] Number or "
        "points in source (%lu) differs than target (%lu)!\n",
        indices_src.size(), cloud_tgt.points.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void pcl::registration::TransformationEstimationTranslationOnly<
    PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource> &cloud_src,
                                const std::vector<int> &indices_src,
                                const pcl::PointCloud<PointTarget> &cloud_tgt,
                                const std::vector<int> &indices_tgt,
                                Matrix4 &transformation_matrix) const {
  if (indices_src.size() != indices_tgt.size()) {
    PCL_ERROR(
        "[pcl::TransformationEstimationTranslationOnly::"
        "estimateRigidTransformation] Number or points in source (%lu) differs "
        "than target (%lu)!\n",
        indices_src.size(), indices_tgt.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, indices_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void pcl::registration::TransformationEstimationTranslationOnly<
    PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource> &cloud_src,
                                const pcl::PointCloud<PointTarget> &cloud_tgt,
                                const pcl::Correspondences &correspondences,
                                Matrix4 &transformation_matrix) const {
  ConstCloudIterator<PointSource> source_it(cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences, false);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void pcl::registration::TransformationEstimationTranslationOnly<
    PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(ConstCloudIterator<PointSource> &source_it,
                                ConstCloudIterator<PointTarget> &target_it,
                                Matrix4 &transformation_matrix) const {
  // Init the algorithm
  source_it.reset();
  target_it.reset();
  transformation_matrix.setIdentity();

  // Calculate the centroids
  Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
  compute3DCentroid(source_it, centroid_src);
  compute3DCentroid(target_it, centroid_tgt);

  // Calculate the translation
  transformation_matrix.block(0, 3, 3, 1) =
      centroid_tgt.head(3) - centroid_src.head(3);
}

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_ONLY_HPP_ */
