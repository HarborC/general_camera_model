#include "general_camera_model/general_camera_model.hpp"
#include "general_camera_model/function.hpp"

using namespace general_camera_model;

int main(int argc, char **argv) {
  int width = 1920;
  int height = 1080;
  std::vector<double> params2 = {1.008,    2.710e-4, 2.158e-4, 960, 540,
                                 -867.43,  0,        3.113e-4,
                                 5.142e-8, 2.253e-11};
  // std::vector<double> params2 = {1.004,    2.989e-4, 0.921e-3, 960, 540,
  //                                -877.47,  0,        3.339e-4,
  //                                6.175e-9, 1.104e-11};
  // std::vector<double> params2 = {1.006,    1.794e-4, 5.722e-4, 960, 540,
  //                                -875.98,  0,        3.358e-4,
  //                                2.055e-8, 2.877e-11};
  std::vector<double> params(SCARAMUZZA_PARAMS_SIZE, 0);
  for (int i = 0; i < params2.size(); i++) {
    params[i] = params2[i];
  }

  GeneralCameraModel camera_model1;
  camera_model1.cameraModelInitializeParams(10, 0, width, height);
  camera_model1.setParams(params);
  camera_model1.setCameraName("cam0");
  camera_model1.generateExtraParams();
  camera_model1.saveToTxtFile(std::string(TEST_DATA_DIR) + "/penn_c1_cam0.txt");
  camera_model1.saveToCerealJsonFile(std::string(TEST_DATA_DIR) +
                                     "/penn_c1_cam0.json");

  std::string img_path = std::string(TEST_DATA_DIR) + "/2049.png";
  std::string out_path = std::string(TEST_DATA_DIR) + "/2049_undistort.png";
  std::string out_path2 = std::string(TEST_DATA_DIR) + "/2049_undistort2.png";

  cv::Mat img = cv::imread(img_path);

  {
    std::pair<cv::Mat, cv::Mat> remaps = initUndistortRectifyMap(camera_model1);
    cv::Mat img_undistort;
    cv::remap(img, img_undistort, remaps.first, remaps.second, cv::INTER_AREA);
    cv::imwrite(out_path, img_undistort);
  }

  GeneralCameraModel camera_model2 = cameraModelConvert(camera_model1, 3);

  {
    std::pair<cv::Mat, cv::Mat> remaps = initUndistortRectifyMap(camera_model2);
    cv::Mat img_undistort;
    cv::remap(img, img_undistort, remaps.first, remaps.second, cv::INTER_AREA);
    cv::imwrite(out_path2, img_undistort);
  }

  camera_model2.saveToCerealJsonFile(std::string(TEST_DATA_DIR) +
                                     "/2penn_c1_cam0.json");

  return 0;
}