#ifndef MG_FEATURE_H
#define MG_FEATURE_H

#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/search/impl/kdtree.hpp>

namespace pcl {
/**
 * @brief Type for an opencv feature descriptor vector.
 */
typedef Eigen::Matrix<unsigned char, 32, 1> Descriptor8U;
/**
 * @brief Point type for the Mixed Gaussian features.
 */
struct MGFeature {
  PCL_ADD_POINT4D;              /*!< Mean x,y,z */
  float sigma[9];               /*!< Covariance matrix */
  unsigned char descriptor[32]; /*!< Descriptor of the feature */
  /**
   * @brief Get the covariance matrix as an Eigen matrix.
   * @return Covariance matrix as an Eigen Matrix3f.
   */
  inline Eigen::Map<Eigen::Matrix3f> getSigmaMatrix3f() {
    return (Eigen::Matrix3f::Map(sigma));
  }
  /**
   * @brief Get the covariance matrix as an Eigen matrix.
   * @return Covariance matrix as an Eigen Matrix3f.
   */
  inline const Eigen::Map<const Eigen::Matrix3f> getSigmaMatrix3f() const {
    return (Eigen::Matrix3f::Map(sigma));
  }
  /**
   * @brief Get the descriptor of the feature as an Eigen vector.
   * @return Vector containing the descriptor.
   */
  inline Eigen::Map<Descriptor8U> getDescriptor8U() {
    return (Descriptor8U::Map(descriptor));
  }
  /**
   * @brief Get the descriptor of the feature as an Eigen vector.
   * @return Vector containing the descriptor.
   */
  inline const Eigen::Map<const Descriptor8U> getDescriptor8U() const {
    return (Descriptor8U::Map(descriptor));
  }
  // Align new allocator
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

// Register the new point type
POINT_CLOUD_REGISTER_POINT_STRUCT(
    MGFeature, (float, x, x)(float, y, y)(float, z, z)(float, sigma, sigma)(
                   unsigned char, descriptor, descriptor));

// Useful typedefs
typedef pcl::MGFeature MGFeature;
typedef pcl::PointCloud<MGFeature> MGFeatureCloud;
typedef pcl::PointCloud<MGFeature>::Ptr MGFeatureCloudPtr;

#endif /*MG_FEATURE_H*/
