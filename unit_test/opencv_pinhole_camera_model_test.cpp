#include <gtest/gtest.h>

#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

TEST(OpenCVPinholeCameraModel, cameraModelInitializeParams) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(2, 800, 1600, 800);
  EXPECT_EQ(camera_model.modelId(), 2);
  EXPECT_EQ(camera_model.numParams(), 8);

  std::vector<double> params = camera_model.params();

  EXPECT_EQ(params[0], 800);
  EXPECT_EQ(params[1], 800);
  EXPECT_EQ(params[2], 800);
  EXPECT_EQ(params[3], 400);
  EXPECT_EQ(params[4], 0);
  EXPECT_EQ(params[5], 0);
  EXPECT_EQ(params[6], 0);
  EXPECT_EQ(params[7], 0);
}

TEST(OpenCVPinholeCameraModel, spaceToPlane) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(2, 800, 1600, 800);

  Eigen::Vector3d P(0.125, 0, 1);
  Eigen::Vector2d p;
  camera_model.spaceToPlane(P, &p);

  // std::cout << "p: " << p << std::endl;

  EXPECT_LE(std::fabs(p(0) - 900), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 400), 1e-6);
}

TEST(OpenCVPinholeCameraModel, planeToSpace) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(2, 800, 1600, 800);

  Eigen::Vector2d p(900, 400);
  Eigen::Vector3d P;
  camera_model.planeToSpace(p, &P);

  // std::cout << "P: " << P << std::endl;

  EXPECT_LE(std::fabs(P(0) - 0.125), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 1), 1e-6);
}

TEST(OpenCVPinholeCameraModel, SaveAndLoad) {
  GeneralCameraModel camera_model;
  camera_model.cameraModelInitializeParams(2, 800, 1600, 800);
  EXPECT_EQ(camera_model.modelId(), 2);
  camera_model.saveToTxtFile(std::string(TEST_DATA_DIR) +
                             "/camera_model_opencv_pinhole.txt");

  GeneralCameraModel camera_model1;
  camera_model1.loadFromTxtFile(std::string(TEST_DATA_DIR) +
                                "/camera_model_opencv_pinhole.txt");
  EXPECT_EQ(camera_model1.modelId(), 2);
  EXPECT_EQ(camera_model1.numParams(), 8);
  EXPECT_EQ(camera_model1.width(), 1600);
  EXPECT_EQ(camera_model1.height(), 800);
  std::vector<double> params = camera_model.params();
  EXPECT_EQ(params[0], 800);
  EXPECT_EQ(params[1], 800);
  EXPECT_EQ(params[2], 800);
  EXPECT_EQ(params[3], 400);
  EXPECT_EQ(params[4], 0);
  EXPECT_EQ(params[5], 0);
  EXPECT_EQ(params[6], 0);
  EXPECT_EQ(params[7], 0);
}

TEST(OpenCVPinholeCameraModel, doubleCheck) {
  GeneralCameraModel camera_model;
  camera_model.cameraModelInitializeParams(2, 800, 1600, 800);

  std::vector<double> params = {799.1,  801.1, 798.6,   402.1,
                                0.0112, 0.012, 0.00113, 0.00122};
  camera_model.setParams(params);

  std::vector<double> params2 = camera_model.params();
  EXPECT_EQ(params2[0], 799.1);
  EXPECT_EQ(params2[1], 801.1);
  EXPECT_EQ(params2[2], 798.6);
  EXPECT_EQ(params2[3], 402.1);
  EXPECT_EQ(params2[4], 0.0112);
  EXPECT_EQ(params2[5], 0.012);
  EXPECT_EQ(params2[6], 0.00113);
  EXPECT_EQ(params2[7], 0.00122);

  for (int i = 0; i < 1600; i += 200) {
    for (int j = 0; j < 800; j += 200) {
      Eigen::Vector2d p(i, j);
      Eigen::Vector3d P;
      camera_model.planeToSpace(p, &P);

      Eigen::Vector2d p1;
      camera_model.spaceToPlane(P, &p1);
      // std::cout << "p: " << p.transpose() << ", p1: " << p1.transpose() <<
      // std::endl;
      EXPECT_LE((p - p1).norm(), 0.5);
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}