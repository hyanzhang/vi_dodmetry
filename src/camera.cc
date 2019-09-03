#include "vi_odometry/camera.h"
#include "vi_odometry/config.h"
namespace vi_odometry
{
Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}
Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &position_in_world, const Sophus::SE3<double> &T_c_w)
{
    return T_c_w * position_in_world;
}
Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &position_in_camera, const Sophus::SE3<double> &T_c_w)
{
    return T_c_w.inverse() * position_in_camera;
}
Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &position_in_camera)
{
    /**
     * There are acturally two steps to convert a position in camera frame to image frame:
     * 1.convert a 3D point in camera frame to the plane there z equals to 1.[x, y, z] -> [x/z, y/z, 1]; 
     * 2.convert the point to image frame (pin-hole camera model)
     */
    Eigen::Vector2d position_in_image;
    position_in_image(0, 0) = fx_ * position_in_camera(0, 0) / position_in_camera(2, 0) + cx_;
    position_in_image(1, 0) = fy_ * position_in_camera(1, 0) / position_in_camera(2, 0) + cy_;
    return position_in_image;
}
Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &position_in_image, double depth)
{
    Eigen::Vector3d position_in_camera;
    position_in_camera(2, 0) = depth;
    position_in_camera(0, 0) = (position_in_image(0, 0) - cx_) * depth / fx_;
    position_in_camera(1, 0) = (position_in_image(1, 0) - cy_) * depth / fy_;
    return position_in_camera;
}
Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d &position_in_image, const Sophus::SE3<double> &T_c_w, double depth)
{
    return T_c_w.inverse() * pixel2camera(position_in_image, depth);
}
Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d &position_in_world, const Sophus::SE3<double> &T_c_w)
{
    return camera2pixel(world2camera(position_in_world, T_c_w));
}
} // namespace vi_odometry
