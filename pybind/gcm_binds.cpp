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

  py::class_<GeneralCameraModel>(m, "CameraModel")
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
      .def("initialize_params", &GeneralCameraModel::cameraModelInitializeParams)
      .def("plane_to_space", &GeneralCameraModel::planeToSpace2)
      .def("space_to_plane", &GeneralCameraModel::spaceToPlane2)
      .def("generate_extra_params", &GeneralCameraModel::generateExtraParams)
      .def("mask", &GeneralCameraModel::mask)
      .def("has_mask", &GeneralCameraModel::hasMask)
      .def("set_mask", &GeneralCameraModel::setMask)
      .def("is_pixel_valid", &GeneralCameraModel::isPixelVaild);

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