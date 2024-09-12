#include <gtest/gtest.h>

#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

TEST(OpenCVFisheyeCameraModel, cameraModelInitializeParams) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(3, 800, 1600, 800);
  EXPECT_EQ(camera_model.modelId(), 3);
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

TEST(OpenCVFisheyeCameraModel, spaceToPlane) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(3, 800, 1600, 800);

  Eigen::Vector3d P(0, 0, 1);
  Eigen::Vector2d p;
  camera_model.spaceToPlane(P, &p);

  // std::cout << "p: " << p << std::endl;

  EXPECT_LE(std::fabs(p(0) - 800), 1e-6);
  EXPECT_LE(std::fabs(p(1) - 400), 1e-6);
}

TEST(OpenCVFisheyeCameraModel, planeToSpace) {
  GeneralCameraModel camera_model;
  // initialize SimplePinholeCameraModel class
  camera_model.cameraModelInitializeParams(3, 800, 1600, 800);

  Eigen::Vector2d p(800, 400);
  Eigen::Vector3d P;
  camera_model.planeToSpace(p, &P);

  // std::cout << "P: " << P << std::endl;

  EXPECT_LE(std::fabs(P(0) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(1) - 0), 1e-6);
  EXPECT_LE(std::fabs(P(2) - 1), 1e-6);
}

TEST(OpenCVFisheyeCameraModel, SaveAndLoad) {
  GeneralCameraModel camera_model;
  camera_model.cameraModelInitializeParams(3, 800, 1600, 800);
  EXPECT_EQ(camera_model.modelId(), 3);
  camera_model.saveToTxtFile(std::string(TEST_DATA_DIR) +
                             "/camera_model_opencv_fisheye.txt");

  GeneralCameraModel camera_model1;
  camera_model1.loadFromTxtFile(std::string(TEST_DATA_DIR) +
                                "/camera_model_opencv_fisheye.txt");
  EXPECT_EQ(camera_model1.modelId(), 3);
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

TEST(OpenCVFisheyeCameraModel, CerealSaveAndLoad) {
  GeneralCameraModel camera_model;
  camera_model.cameraModelInitializeParams(3, 800, 1600, 800);
  EXPECT_EQ(camera_model.modelId(), 3);
  camera_model.saveToCerealJsonFile(std::string(TEST_DATA_DIR) +
                                    "/camera_model_opencv_fisheye.json");

  GeneralCameraModel camera_model1;
  camera_model1.loadFromCerealJsonFile(std::string(TEST_DATA_DIR) +
                                       "/camera_model_opencv_fisheye.json");
  EXPECT_EQ(camera_model1.modelId(), 3);
  EXPECT_EQ(camera_model1.numParams(), 8);
  EXPECT_EQ(camera_model1.width(), 1600);
  EXPECT_EQ(camera_model1.height(), 800);
  std::vector<double> params = camera_model1.params();
  EXPECT_EQ(params[0], 800);
  EXPECT_EQ(params[1], 800);
  EXPECT_EQ(params[2], 800);
  EXPECT_EQ(params[3], 400);
  EXPECT_EQ(params[4], 0);
  EXPECT_EQ(params[5], 0);
  EXPECT_EQ(params[6], 0);
  EXPECT_EQ(params[7], 0);

  GeneralCameraModel camera_model2;
  camera_model2.cameraModelInitializeParams(3, 800, 700, 800);
  EXPECT_EQ(camera_model2.modelId(), 3);
  camera_model2.saveToCerealBinaryFile(std::string(TEST_DATA_DIR) +
                                       "/camera_model_opencv_fisheye.bin");

  GeneralCameraModel camera_model3;
  camera_model3.loadFromCerealBinaryFile(std::string(TEST_DATA_DIR) +
                                         "/camera_model_opencv_fisheye.bin");
  EXPECT_EQ(camera_model3.modelId(), 3);
  EXPECT_EQ(camera_model3.numParams(), 8);
  EXPECT_EQ(camera_model3.width(), 700);
  EXPECT_EQ(camera_model3.height(), 800);
  std::vector<double> params2 = camera_model3.params();
  EXPECT_EQ(params2[0], 800);
  EXPECT_EQ(params2[1], 800);
  EXPECT_EQ(params2[2], 350);
  EXPECT_EQ(params2[3], 400);
  EXPECT_EQ(params2[4], 0);
  EXPECT_EQ(params2[5], 0);
  EXPECT_EQ(params2[6], 0);
  EXPECT_EQ(params2[7], 0);
}

TEST(OpenCVFisheyeCameraModel, doubleCheck) {
  // int width = 1280;
  // int height = 1024;
  // double f = 740;
  // std::vector<double> params = {739.1654756101043, 739.1438452683457,
  // 625.826167006398, 517.3370973594253, 0.019327620961435945,
  // 0.006784242994724914, -0.008658628531456217, 0.0051893686731546585};

  int width = 3000;
  int height = 3000;
  double f = 1040;
  std::vector<double> params = {1040.223693239266,      1040.480053368926,
                                1491.7297444959622,     1590.8641845289355,
                                -0.0680387534420108,    0.0007178583402205706,
                                -0.0021979952320485335, 0.0005117738387110172};

  GeneralCameraModel camera_model;
  camera_model.cameraModelInitializeParams(3, f, width, height);
  camera_model.setParams(params);

  std::vector<double> params2 = camera_model.params();
  EXPECT_EQ(params2[0], params[0]);
  EXPECT_EQ(params2[1], params[1]);
  EXPECT_EQ(params2[2], params[2]);
  EXPECT_EQ(params2[3], params[3]);
  EXPECT_EQ(params2[4], params[4]);
  EXPECT_EQ(params2[5], params[5]);
  EXPECT_EQ(params2[6], params[6]);
  EXPECT_EQ(params2[7], params[7]);

  for (int i = 0; i < width; i += 200) {
    for (int j = 0; j < height; j += 200) {
      Eigen::Vector2d p(i, j);
      Eigen::Vector3d P;
      camera_model.planeToSpace(p, &P);

      Eigen::Vector2d p1;
      camera_model.spaceToPlane(P, &p1);
      // std::cout << "P: " << P.transpose() << ", p: " << p.transpose() << ",
      // p1: " << p1.transpose() << std::endl;
      EXPECT_LE((p - p1).norm(), 0.5);
    }
  }
}

TEST(OpenCVFisheyeCameraModel, doubleCheck2) {
  int width = 3000;
  int height = 3000;
  double f = 1040;
  std::vector<double> params = {1040.223693239266,      1040.480053368926,
                                1491.7297444959622,     1590.8641845289355,
                                -0.0680387534420108,    0.0007178583402205706,
                                -0.0021979952320485335, 0.0005117738387110172};

  GeneralCameraModel camera_model;
  camera_model.cameraModelInitializeParams(3, f, width, height);
  camera_model.setParams(params);

  for (int i = 0; i < width; i += 200) {
    for (int j = 0; j < height; j += 200) {
      Eigen::Vector2d p(i, j);

      Eigen::Vector3d P2;
      // other method
      {
        const double f1 = params[0];
        const double f2 = params[1];
        const double c1 = params[2];
        const double c2 = params[3];

        double x = p(0);
        double y = p(1);

        // Lift points to normalized plane
        double u = (x - c1) / f1;
        double v = (y - c2) / f2;

        // 可能不能处理超过180度的情况
        opencv_fisheye_functions::iterativeUndistortion(&params[4], &u, &v);

        P2(0) = u;
        P2(1) = v;
        P2(2) = 1.0;
        P2.normalize();
      }

      Eigen::Vector3d P1;
      camera_model.planeToSpace(p, &P1);
      if (P1(2) > 0) {
        EXPECT_LE((P2 - P1).norm(), 0.5);
      }

      Eigen::Vector2d p2;
      // other method
      {
        const double f1 = params[0];
        const double f2 = params[1];
        const double c1 = params[2];
        const double c2 = params[3];

        double u = P1(0) / P1(2);
        double v = P1(1) / P1(2);

        // Distortion
        double du, dv;
        opencv_fisheye_functions::distortion(&params[4], u, v, &du, &dv);
        p2(0) = u + du;
        p2(1) = v + dv;

        // Transform to image coordinates
        p2(0) = f1 * p2(0) + c1;
        p2(1) = f2 * p2(1) + c2;
      }

      Eigen::Vector2d p1;
      camera_model.spaceToPlane(P1, &p1);
      if (P1(2) > 0) {
        EXPECT_LE((p2 - p1).norm(), 0.5);
      }
      EXPECT_LE((p - p1).norm(), 0.5);

      // std::cout << "p: " << p.transpose() << ", P1: " << P1.transpose()
      //           << ", P2: " << P2.transpose() << ", p1: " << p1.transpose()
      //           << ", p2: " << p2.transpose() << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}