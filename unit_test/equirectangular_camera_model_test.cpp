#include <gtest/gtest.h>

#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

TEST(EquirectangularCameraModel, cameraModelInitializeParams) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(1, 800, 1600, 800);
  EXPECT_EQ(camera_model.modelId(), 1);
  EXPECT_EQ(camera_model.numParams(), 3);

  std::vector<double> params = camera_model.params();

  EXPECT_EQ(params[0], 1600);
  EXPECT_EQ(params[1], 800);
  EXPECT_EQ(params[2], 20);
}

TEST(EquirectangularCameraModel, spaceToPlane) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(1, 800, 1600, 800);

  Eigen::Vector3d P(0, 1, 0);
  Eigen::Vector2d p;
  camera_model.spaceToPlane(P, &p);

  // std::cout << "p: " << p << std::endl;

  EXPECT_LE(std::fabs(p(0) - 799.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 799), 1e-6);
}

TEST(EquirectangularCameraModel, planeToSpace) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(1, 800, 1600, 800);

  Eigen::Vector2d p(799.5, 799);
  Eigen::Vector3d P;
  camera_model.planeToSpace(p, &P);

  // std::cout << "P: " << P << std::endl;

  EXPECT_LE(std::fabs(P(0) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 1), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 0), 1e-6);
}

TEST(EquirectangularCameraModel, SaveAndLoad) {
  GeneralCameraModel camera_model;
  camera_model.cameraModelInitializeParams(1, 800, 1600, 800);
  EXPECT_EQ(camera_model.modelId(), 1);
  camera_model.saveToTxtFile(std::string(TEST_DATA_DIR) +
                             "/camera_model_equirectangular.txt");

  GeneralCameraModel camera_model1;
  camera_model1.loadFromTxtFile(std::string(TEST_DATA_DIR) +
                                "/camera_model_equirectangular.txt");
  EXPECT_EQ(camera_model1.modelId(), 1);
  EXPECT_EQ(camera_model1.numParams(), 3);
  EXPECT_EQ(camera_model1.width(), 1600);
  EXPECT_EQ(camera_model1.height(), 800);
  std::vector<double> params = camera_model.params();
  EXPECT_EQ(params[0], 1600);
  EXPECT_EQ(params[1], 800);
  EXPECT_EQ(params[2], 20);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}