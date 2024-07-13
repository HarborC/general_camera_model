#include <gtest/gtest.h>

#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

TEST(SimplePinholeCameraModel, cameraModelInitializeParams) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(0, 800, 800, 800);

  std::vector<double> params = camera_model.params();

  EXPECT_EQ(params[0], 800);
  EXPECT_EQ(params[1], 400);
  EXPECT_EQ(params[2], 400);
}

TEST(SimplePinholeCameraModel, spaceToPlane) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(0, 800, 800, 800);

  Eigen::Vector3d P(1, 1, 1);
  Eigen::Vector2d p;
  camera_model.spaceToPlane(P, &p);

  // std::cout << "p: " << p << std::endl;

  EXPECT_LE(std::fabs(p(0) - 1200), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 1200), 1e-6);
}

TEST(SimplePinholeCameraModel, planeToSpace) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(0, 800, 800, 800);

  Eigen::Vector2d p(200, 200);
  Eigen::Vector3d P;
  camera_model.planeToSpace(p, &P);

  // std::cout << "P: " << P << std::endl;

  EXPECT_LE(std::fabs(P(0) + 0.25), 1e-6);
  EXPECT_LE(std::fabs(P(1) + 0.25), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 1), 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}