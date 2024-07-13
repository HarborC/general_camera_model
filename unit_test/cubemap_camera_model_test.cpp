#include <gtest/gtest.h>

#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

TEST(CubeMapCameraModel, cameraModelInitializeParams) {
  GeneralCameraModel camera_model;
  // initialize CubeMapCameraModel class
  int f = 500;
  camera_model.cameraModelInitializeParams(5, f, f * 8, f * 6);
  EXPECT_EQ(camera_model.modelId(), 5);
  EXPECT_EQ(camera_model.numParams(), 1);

  std::vector<double> params = camera_model.params();

  EXPECT_EQ(params[0], 1000);
}

TEST(CubeMapCameraModel, spaceToPlane) {
  GeneralCameraModel camera_model;
  // initialize CubeMapCameraModel class
  int f = 500;
  camera_model.cameraModelInitializeParams(5, f, f * 8, f * 6);

  Eigen::Vector3d P;
  Eigen::Vector2d p;

  // example 1
  P << 0, 1, 0;
  camera_model.spaceToPlane(P, &p);
  EXPECT_LE(std::fabs(p(0) - 1500 + 0.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 2500 + 0.5), 1e-6);

  // example 2
  P << 0, 0, 1;
  camera_model.spaceToPlane(P, &p);
  EXPECT_LE(std::fabs(p(0) - 1500 + 0.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 1500 + 0.5), 1e-6);

  // example 3
  P << 1, 0, 0;
  camera_model.spaceToPlane(P, &p);
  EXPECT_LE(std::fabs(p(0) - 2500 + 0.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 1500 + 0.5), 1e-6);

  // example 4
  P << 0, -1, 0;
  camera_model.spaceToPlane(P, &p);
  EXPECT_LE(std::fabs(p(0) - 1500 + 0.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 500 + 0.5), 1e-6);

  // example 5
  P << -1, 0, 0;
  camera_model.spaceToPlane(P, &p);
  EXPECT_LE(std::fabs(p(0) - 500 + 0.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 1500 + 0.5), 1e-6);

  // example 6
  P << 0, 0, -1;
  camera_model.spaceToPlane(P, &p);
  EXPECT_LE(std::fabs(p(0) - 3500 + 0.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 1500 + 0.5), 1e-6);

  // example 7
  P << -0.5, -0.5, 0.5;
  camera_model.spaceToPlane(P, &p);
  EXPECT_LE(std::fabs(p(0) - 1000 + 0.5), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 1000 + 0.5), 1e-6);
}

TEST(CubeMapCameraModel, planeToSpace) {
  GeneralCameraModel camera_model;
  // initialize CubeMapCameraModel class
  int f = 500;
  camera_model.cameraModelInitializeParams(5, f, f * 8, f * 6);

  Eigen::Vector2d p;
  Eigen::Vector3d P;

  // example 1
  p << 1500 - 0.5, 2500 - 0.5;
  camera_model.planeToSpace(p, &P);
  EXPECT_LE(std::fabs(P(0) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 1), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 0), 1e-6);

  // example 2
  p << 1500 - 0.5, 1500 - 0.5;
  camera_model.planeToSpace(p, &P);
  EXPECT_LE(std::fabs(P(0) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 1), 1e-6);

  // example 3
  p << 2500 - 0.5, 1500 - 0.5;
  camera_model.planeToSpace(p, &P);
  EXPECT_LE(std::fabs(P(0) - 1), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 0), 1e-6);

  // example 4
  p << 1500 - 0.5, 500 - 0.5;
  camera_model.planeToSpace(p, &P);
  EXPECT_LE(std::fabs(P(0) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(1) + 1), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 0), 1e-6);

  // example 5
  p << 500 - 0.5, 1500 - 0.5;
  camera_model.planeToSpace(p, &P);
  EXPECT_LE(std::fabs(P(0) + 1), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 0), 1e-6);

  // example 6
  p << 3500 - 0.5, 1500 - 0.5;
  camera_model.planeToSpace(p, &P);
  EXPECT_LE(std::fabs(P(0) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(2) + 1), 1e-6);

  // example 7
  p << 1000 - 0.5, 1000 - 0.5;
  camera_model.planeToSpace(p, &P);
  std::cout << P << std::endl;
  EXPECT_LE(std::fabs(P(0) + 1 / std::sqrt(3)), 1e-6);
  EXPECT_LE(std::fabs(P(1) + 1 / std::sqrt(3)), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 1 / std::sqrt(3)), 1e-6);
}

TEST(CubeMapCameraModel, SaveAndLoad) {
  GeneralCameraModel camera_model;
  int f = 500;
  camera_model.cameraModelInitializeParams(5, f, f * 8, f * 6);
  EXPECT_EQ(camera_model.modelId(), 5);
  camera_model.saveToTxtFile(std::string(TEST_DATA_DIR) +
                             "/camera_model_cubemap.txt");

  GeneralCameraModel camera_model1;
  camera_model1.loadFromTxtFile(std::string(TEST_DATA_DIR) +
                                "/camera_model_cubemap.txt");
  EXPECT_EQ(camera_model1.modelId(), 5);
  EXPECT_EQ(camera_model1.numParams(), 1);
  EXPECT_EQ(camera_model1.width(), 4000);
  EXPECT_EQ(camera_model1.height(), 3000);
  std::vector<double> params = camera_model.params();
  EXPECT_EQ(params[0], 1000);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}