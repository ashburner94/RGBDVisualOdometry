RGBD visual odometry
====================

ROS node for a visual odometry with an RGBD camera like the Microsoft Kinect or the Intel Realsense.

Note that the greyscale image and the depth image
must be "the same image", so a pixel (u, v) in the greyscale image has the depth value
z = depth_image(u, v).
This node is based on the works of Ivan Dryanovski et. al "Fast Visual Odometry and Mapping from RGB-D
Data" and "An accurate closed-form estimate of ICP's covariance" by A. Censi.
