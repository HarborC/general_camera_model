#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

int main(int argc, char **argv) {
  std::string img_path = std::string(PROJECT_DIR) + "/apps/test_data/cam0.jpg";
  std::string camera_path = std::string(PROJECT_DIR) + "/apps/test_data/cam0.txt";

  cv::Mat img = cv::imread(img_path);

  GeneralCameraModel camera_model1;
  camera_model1.loadConfigFile(camera_path);

  // CubeMapCameraModel
  {
    int ww = 2000;
    GeneralCameraModel camera_model2;
    camera_model2.cameraModelInitializeParams(5, ww / 2.0, 4 * ww, 3 * ww);

    cv::Mat remap_x, remap_y;
    camera_model1.initUndistortRectifyMap(
        camera_model2, Eigen::Matrix3d::Identity(), &remap_x, &remap_y);

    cv::Mat img_undistort;
    cv::remap(img, img_undistort, remap_x, remap_y, cv::INTER_AREA);
    std::string save_path = std::string(PROJECT_DIR) + "/apps/test_data/cube_map_result.jpg";
    cv::imwrite(save_path, img_undistort);
  }

  // SimplePinholeCameraModel
  {
    int ww = 1500;
    GeneralCameraModel camera_model2;
    camera_model2.cameraModelInitializeParams(0, ww, ww * 2, ww * 2);

    cv::Mat remap_x, remap_y;
    camera_model1.initUndistortRectifyMap(
        camera_model2, Eigen::Matrix3d::Identity(), &remap_x, &remap_y);

    cv::Mat img_undistort;
    cv::remap(img, img_undistort, remap_x, remap_y, cv::INTER_AREA);
    std::string save_path = std::string(PROJECT_DIR) + "/apps/test_data/simple_result.jpg";
    cv::imwrite(save_path, img_undistort);
  }

  // EquirectangularCameraModel
  {
    int ww = 2000;
    GeneralCameraModel camera_model2;
    camera_model2.cameraModelInitializeParams(1, ww, ww * 2, ww);

    cv::Mat remap_x, remap_y;
    camera_model1.initUndistortRectifyMap(
        camera_model2, Eigen::Matrix3d::Identity(), &remap_x, &remap_y);

    cv::Mat img_undistort;
    cv::remap(img, img_undistort, remap_x, remap_y, cv::INTER_AREA);
    std::string save_path = std::string(PROJECT_DIR) + "/apps/test_data/equirectangular_result.jpg";
    cv::imwrite(save_path, img_undistort);
  }

  return 0;
}