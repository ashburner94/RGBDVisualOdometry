cmake_minimum_required(VERSION 2.8.3)

## Set the correct C++ standard
set (CMAKE_CXX_STANDARD 11)

## Treat all warnings as errors
add_compile_options(-Werror -Wall -Wextra -Wno-deprecated-declarations)

## Turn on compile time optimizations
if(${CMAKE_BUILD_TYPE} MATCHES "Release")
  add_compile_options(-O3)
endif()

## Export the compiler commands list
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Point cloud library
find_package(PCL 1.7 REQUIRED COMPONENTS
  common
  kdtree
  registration
  search
)

## Eigen
find_package(Eigen3 REQUIRED)

## OpenCV
find_package(OpenCV 3 REQUIRED)

###########
## Build ##
###########

## Additional Resources
include_directories(
  include/
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

## Visual Odometry lib
add_library(rgbd_visual_odometry_lib
  src/rgbd_visual_odometry.cpp
  src/environment_model.cpp
  src/correspondence_estimation_mahalanobis.cpp
  src/correspondence_estimation_mahalanobis_with_descriptor_distances.cpp
  src/icp_mg.cpp
  src/mg_feature.cpp
  src/transformation_estimator.cpp
  src/data_cloud_generator.cpp
)

## Link the rgbd visual odometry lib
target_link_libraries(rgbd_visual_odometry_lib
  ${PCL_LIBRARIES}
  ${OPENCV_LIBRARIES}
)
