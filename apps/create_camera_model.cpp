#include "general_camera_model/general_camera_model.hpp"
using namespace general_camera_model;

int main(int argc, char **argv) {
    int width = 1280;
    int height = 1024;
    double f = 740;
    std::vector<double> params = {743.4286936207343,      743.5545205462922,
                                    618.7186883884866,     506.7275058699658,
                                    0.023584346301328347,    -0.006764098468377487,
                                    0.010259071387776937, -0.0037561745737771414};

    GeneralCameraModel camera_model1;
    camera_model1.cameraModelInitializeParams(3, f, width, height);
    camera_model1.setParams(params);
    camera_model1.setCameraName("cam0");
    camera_model1.saveToTxtFile("/home/jiagang.chen/a/general_camera_model/datatset_configs/rs_tum/cam0.txt");
    camera_model1.saveToCerealJsonFile("/home/jiagang.chen/a/general_camera_model/datatset_configs/rs_tum/cam0.json");
    camera_model1.saveToCerealBinaryFile("/home/jiagang.chen/a/general_camera_model/datatset_configs/rs_tum/cam0.bin");

    width = 1280;
    height = 1024;
    f = 740;
    params = {739.1654756101043, 739.1438452683457, 625.826167006398, 517.3370973594253, 0.019327620961435945, 0.006784242994724914, -0.008658628531456217, 0.0051893686731546585};

    GeneralCameraModel camera_model2;
    camera_model2.cameraModelInitializeParams(3, f, width, height);
    camera_model2.setParams(params);
    camera_model2.setCameraName("cam1");
    camera_model2.saveToTxtFile("/home/jiagang.chen/a/general_camera_model/datatset_configs/rs_tum/cam1.txt");
    camera_model2.saveToCerealJsonFile("/home/jiagang.chen/a/general_camera_model/datatset_configs/rs_tum/cam1.json");
    camera_model2.saveToCerealBinaryFile("/home/jiagang.chen/a/general_camera_model/datatset_configs/rs_tum/cam1.bin");

    return 0;
}

