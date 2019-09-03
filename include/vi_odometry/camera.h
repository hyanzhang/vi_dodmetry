#ifndef _CAMERA_H_
#define _CAMERA_H_
#include "vi_odometry/common_headers.h"

namespace vi_odometry
{
class Camera
{
public:
    std::shared_ptr<Camera> Ptr;
    float fx_, fy_, cx_, cy_, depth_scale_; //camera intrinsics

    Camera();
    Camera(float fx, float fy, float cx, float cy, float depth_scale) : fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {}
    Eigen::Vector3d world2camera(const Eigen::Vector3d &position_in_world, const Sophus::SE3<double> &T_c_w);
    Eigen::Vector3d camera2world(const Eigen::Vector3d &position_in_camera, const Sophus::SE3<double> &T_c_w);
    Eigen::Vector2d camera2pixel(const Eigen::Vector3d &position_in_camera);
    Eigen::Vector3d pixel2camera(const Eigen::Vector2d &position_in_image, double depth);
    Eigen::Vector3d pixel2world(const Eigen::Vector2d &position_in_image, const Sophus::SE3<double> &T_c_w, double detpth);
    Eigen::Vector2d world2pixel(const Eigen::Vector3d &position_in_world, const Sophus::SE3<double> &T_c_w);
    ~Camera();
};
} // namespace vi_odometry

#endif