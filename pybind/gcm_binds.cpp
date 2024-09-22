#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "general_camera_model/function.hpp"
#include "general_camera_model/general_camera_model.hpp"
#include "ndarray_converter.h"

namespace py = pybind11;
using namespace general_camera_model;

PYBIND11_MODULE(pygcm, m) {
  NDArrayConverter::init_numpy();

  using namespace pybind11::literals;
  m.doc() = "python interface for pygcm";

  py::class_<GeneralCameraModel, GeneralCameraModel::Ptr>(m, "CameraModel")
      .def(py::init<>())
      .def("load_config_file", &GeneralCameraModel::loadConfigFile)
      .def("save_config_file", &GeneralCameraModel::saveConfigFile)
      .def("info", &GeneralCameraModel::info)
      .def("camera_name", &GeneralCameraModel::cameraName)
      .def("set_camera_name", &GeneralCameraModel::setCameraName)
      .def("model_id", &GeneralCameraModel::modelId)
      .def("model_name", &GeneralCameraModel::modelName)
      .def("set_model_id", &GeneralCameraModel::setModelId)
      .def("set_model_id_from_name", &GeneralCameraModel::setModelIdFromName)
      .def("width", &GeneralCameraModel::width)
      .def("height", &GeneralCameraModel::height)
      .def("set_width", &GeneralCameraModel::setWidth)
      .def("set_height", &GeneralCameraModel::setHeight)
      .def("num_params", &GeneralCameraModel::numParams)
      .def("params", &GeneralCameraModel::getParams)
      .def("params_value", &GeneralCameraModel::getParamValue)
      .def("set_params", &GeneralCameraModel::setParams)
      .def("params_info", &GeneralCameraModel::paramsInfo)
      .def("initialize_params",
           &GeneralCameraModel::cameraModelInitializeParams)
      .def("plane_to_space", &GeneralCameraModel::planeToSpace2)
      .def("space_to_plane", &GeneralCameraModel::spaceToPlane2)
      .def("generate_extra_params", &GeneralCameraModel::generateExtraParams)
      .def("mask", &GeneralCameraModel::mask)
      .def("has_mask", &GeneralCameraModel::hasMask)
      .def("set_mask",
           static_cast<bool (GeneralCameraModel::*)(std::string mask_path)>(
               &GeneralCameraModel::setMask),
           "set mask from path", py::arg("mask_path"))
      .def("set_mask",
           static_cast<void (GeneralCameraModel::*)(const cv::Mat &mask)>(
               &GeneralCameraModel::setMask),
           "set mask from cv::Mat", py::arg("mask"))
      .def("is_pixel_valid", &GeneralCameraModel::isPixelVaild)
      // 序列化支持
      .def("__getstate__",
           [](const GeneralCameraModel &self) {
             // 返回可序列化的状态，使用pybind11::dict封装
             pybind11::dict state;
             state["camera_name"] = self.cameraName();
             state["model_id"] = self.modelId();
             state["width"] = self.width();
             state["height"] = self.height();
             state["params"] = self.getParams();
             state["mask"] = self.mask();
             return state;
           })
      .def("__setstate__", [](GeneralCameraModel &self, pybind11::dict state) {
        // 从字典恢复对象的状态
        self.setCameraName(state["camera_name"].cast<std::string>());
        self.setModelId(state["model_id"].cast<int>());
        self.setWidth(state["width"].cast<int>());
        self.setHeight(state["height"].cast<int>());
        self.setParams(state["params"].cast<std::vector<double>>());
        self.setMask(state["mask"].cast<cv::Mat>());
      });

  m.def("init_undistort_rectify_map", &initUndistortRectifyMap,
        "A fuction to calculate two camera maps", py::arg("old_camera"),
        py::arg("new_camera") = GeneralCameraModel(),
        py::arg("R") = Eigen::Matrix3d::Identity());

  m.def("calc_overlap_views", &calcOverlapViews, "calculate overlap views",
        py::arg("camera_vec"), py::arg("ex_vec"),
        py::arg("average_depth") = 100, py::arg("ratio") = 0.3,
        py::arg("debug_dir") = "");

  m.def("get_simple_pinhole", &getSimplePinhole,
        "get simple pinhole camera model", py::arg("fx"), py::arg("fy"),
        py::arg("cx"), py::arg("cy"), py::arg("width"), py::arg("height"));

  m.def("camera_model_convert", &cameraModelConvert, "convert camera model",
        py::arg("old_camera"), py::arg("model_id"));

  m.def("batch_plane_to_space", &batchPlaneToSpace,
        "batch plane to space conversion", py::arg("camera"),
        py::arg("plane_pts"));

  m.def("batch_space_to_plane", &batchSpaceToPlane,
        "batch space to plane conversion", py::arg("camera"),
        py::arg("space_pts"));
}