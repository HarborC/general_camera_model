#pragma once
#include "general_camera_model/general_camera_model.hpp"

namespace general_camera_model {

GeneralCameraModel cameraModelConvert(const GeneralCameraModel &old_camera,
                                      int model_id);

std::pair<cv::Mat, cv::Mat> initUndistortRectifyMap(
    const GeneralCameraModel &old_camera,
    const GeneralCameraModel &new_camera = GeneralCameraModel(),
    const Eigen::Matrix3d &R = Eigen::Matrix3d::Identity());

std::vector<std::pair<int, int>>
calcOverlapViews(const std::vector<GeneralCameraModel> &camera_vec,
                 const std::vector<Eigen::Matrix4d> &ex_vec,
                 double average_depth = 100, const double &ratio = 0.3,
                 const std::string &debug_dir = "");

GeneralCameraModel getSimplePinhole(double fx, double fy, double cx, double cy,
                                    int width, int height);

std::vector<Eigen::Vector2d>
batchSpaceToPlane(const GeneralCameraModel &camera,
                  const std::vector<Eigen::Vector3d> &pts);

std::vector<Eigen::Vector3d>
batchPlaneToSpace(const GeneralCameraModel &camera,
                  const std::vector<Eigen::Vector2d> &pts);

} // namespace general_camera_model