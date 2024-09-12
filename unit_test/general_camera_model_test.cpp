#include <gtest/gtest.h>

#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

TEST(GeneralCameraModel, cameraModelInitializeParams) {
  GeneralCameraModel camera_model;
  EXPECT_EQ(camera_model.modelId(), -1);

  camera_model.cameraModelInitializeParams(0, 800, 800, 800);
  EXPECT_EQ(camera_model.modelId(), 0);

  std::vector<double> params = camera_model.params();
  EXPECT_EQ(params[0], 800);
  EXPECT_EQ(params[1], 400);
  EXPECT_EQ(params[2], 400);
}

TEST(GeneralCameraModel, modelName) {
  GeneralCameraModel camera_model;
  EXPECT_EQ(camera_model.modelName(), "");

  camera_model.cameraModelInitializeParams(0, 800, 800, 800);
  EXPECT_EQ(camera_model.modelName(), "SIMPLE_PINHOLE");

  GeneralCameraModel camera_model2;
  camera_model2.setModelId(0);
  EXPECT_EQ(camera_model.modelName(), "SIMPLE_PINHOLE");

  GeneralCameraModel camera_model3;
  camera_model3.setModelIdFromName("SIMPLE_PINHOLE");
  EXPECT_EQ(camera_model.modelId(), 0);
}

TEST(GeneralCameraModel, Params) {
  GeneralCameraModel camera_model;
  EXPECT_EQ(camera_model.numParams(), 0);

  camera_model.cameraModelInitializeParams(0, 800, 800, 800);
  EXPECT_EQ(camera_model.modelName(), "SIMPLE_PINHOLE");

  GeneralCameraModel camera_model2;
  camera_model2.setModelId(0);
  EXPECT_EQ(camera_model.modelName(), "SIMPLE_PINHOLE");

  GeneralCameraModel camera_model3;
  camera_model3.setModelIdFromName("SIMPLE_PINHOLE");
  EXPECT_EQ(camera_model.modelId(), 0);
}

TEST(GeneralCameraModel, SaveAndLoad) {
  GeneralCameraModel camera_model;
  camera_model.saveToTxtFile(std::string(TEST_DATA_DIR) +
                             "/camera_model_invaild.txt");
  camera_model.loadFromTxtFile(std::string(TEST_DATA_DIR) +
                               "/camera_model_invaild.txt");
  EXPECT_EQ(camera_model.modelId(), -1);

  camera_model.cameraModelInitializeParams(0, 800, 500, 400);
  EXPECT_EQ(camera_model.modelId(), 0);
  camera_model.saveToTxtFile(std::string(TEST_DATA_DIR) +
                             "/camera_model_single_pinhole.txt");
  camera_model.loadFromTxtFile(std::string(TEST_DATA_DIR) +
                               "/camera_model_single_pinhole.txt");
  EXPECT_EQ(camera_model.modelId(), 0);
  EXPECT_EQ(camera_model.numParams(), 3);

  GeneralCameraModel camera_model1;
  camera_model1.loadFromTxtFile(std::string(TEST_DATA_DIR) +
                                "/camera_model_single_pinhole.txt");
  EXPECT_EQ(camera_model1.modelId(), 0);
  EXPECT_EQ(camera_model1.numParams(), 3);
  EXPECT_EQ(camera_model1.width(), 500);
  EXPECT_EQ(camera_model1.height(), 400);
  std::vector<double> params = camera_model.params();
  EXPECT_EQ(params[0], 800);
  EXPECT_EQ(params[1], 250);
  EXPECT_EQ(params[2], 200);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}