#include <gtest/gtest.h>

#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

TEST(ScaramuzzaCameraModel, cameraModelInitializeParams) {
  GeneralCameraModel camera_model;
  // initialize ScaramuzzaCameraModel class
  camera_model.cameraModelInitializeParams(10, 800, 1000, 800);

  std::vector<double> params = camera_model.params();

  EXPECT_EQ(params[0], 1.0);
  EXPECT_EQ(params[1], 0);
  EXPECT_EQ(params[2], 0);
  EXPECT_EQ(params[3], 500);
  EXPECT_EQ(params[4], 400);
}

TEST(ScaramuzzaCameraModel, spaceToPlane_and_planeToSpace) {
  int width = 1920;
  int height = 1080;
  std::vector<double> params2 = {1.008,    2.710e-4, 2.158e-4, 960,
                                 540,      -867.43,  0,        3.113e-4,
                                 5.142e-8, 2.253e-11};
  std::vector<double> params(SCARAMUZZA_PARAMS_SIZE, 0);
  for (int i = 0; i < params2.size(); i++) {
    params[i] = params2[i];
  }

  GeneralCameraModel camera_model1;
  camera_model1.cameraModelInitializeParams(10, 0, width, height);
  camera_model1.setParams(params);
  camera_model1.setCameraName("cam0");
  camera_model1.generateExtraParams();

  std::vector<Eigen::Vector3d> Ps = {
      Eigen::Vector3d(1, 1, 1),   Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(-1, 1, 1),  Eigen::Vector3d(1, -1, 1),
      Eigen::Vector3d(-1, -1, 1), Eigen::Vector3d(-1, 0, 1),
      Eigen::Vector3d(1, 0, 1),   Eigen::Vector3d(0, 1, 1),
      Eigen::Vector3d(0, -1, 1)};

  for (int i = 0; i < Ps.size(); i++) {
    std::cout << "idx: " << i << std::endl;

    Eigen::Vector3d P_n = Ps[i].normalized();
    std::cout << "Ps[i]: " << Ps[i].transpose() << std::endl;
    std::cout << "P_n: " << P_n.transpose() << std::endl;

    Eigen::Vector2d p;
    camera_model1.spaceToPlane(Ps[i], &p);

    std::cout << "p: " << p.transpose() << std::endl;

    Eigen::Vector3d P;
    camera_model1.planeToSpace(p, &P);

    std::cout << "P: " << P.transpose() << std::endl;

    EXPECT_LE(std::fabs(P_n(0) - P(0)), 1e-6);
    EXPECT_LE(std::fabs(P_n(1) - P(1)), 1e-6);
    EXPECT_LE(std::fabs(P_n(2) - P(2)), 1e-6);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}