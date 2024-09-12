#pragma once

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>

#include <opencv2/opencv.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

namespace general_camera_model {

static const int kInvalidCameraModelId = -1;

#define SCARAMUZZA_POLY_SIZE 5
#define SCARAMUZZA_INV_POLY_SIZE 12
#define SCARAMUZZA_PARAMS_SIZE                                                 \
  (5 + SCARAMUZZA_POLY_SIZE + SCARAMUZZA_INV_POLY_SIZE)

#ifndef CAMERA_MODEL_DEFINITIONS
#define CAMERA_MODEL_DEFINITIONS(CameraModelClass, model_id_value,             \
                                 model_name_value, num_params_value,           \
                                 params_info_value)                            \
  struct CameraModelClass {                                                    \
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW                                            \
    static const int kModelId = model_id_value;                                \
    static const size_t kNumParams = num_params_value;                         \
    static const int model_id;                                                 \
    static const std::string model_name;                                       \
    static const size_t num_params;                                            \
    static const std::string params_info;                                      \
    static inline std::vector<double>                                          \
    initializeParams(const double focal_length, const size_t width,            \
                     const size_t height);                                     \
    static inline int modelId() { return model_id_value; };                    \
    static inline std::string modelName() { return model_name_value; };        \
    static inline size_t numParams() { return num_params_value; };             \
    static inline std::string paramsInfo() { return params_info_value; };      \
                                                                               \
    template <typename T>                                                      \
    static inline void spaceToPlane(const T *params,                           \
                                    const Eigen::Matrix<T, 3, 1> &P,           \
                                    Eigen::Matrix<T, 2, 1> *p,                 \
                                    Eigen::Matrix<T, 2, 3> *J = nullptr);      \
                                                                               \
    template <typename T>                                                      \
    static inline void planeToSpace(const T *params,                           \
                                    const Eigen::Matrix<T, 2, 1> &p,           \
                                    Eigen::Matrix<T, 3, 1> *P);                \
  };

#endif

#ifndef CAMERA_MODEL_CASES
#define CAMERA_MODEL_CASES                                                     \
  CAMERA_MODEL_CASE(SimplePinholeCameraModel)                                  \
  CAMERA_MODEL_CASE(EquirectangularCameraModel)                                \
  CAMERA_MODEL_CASE(OpenCVPinholeCameraModel)                                  \
  CAMERA_MODEL_CASE(OpenCVFisheyeCameraModel)                                  \
  CAMERA_MODEL_CASE(OmniDirectionalCameraModel)                                \
  CAMERA_MODEL_CASE(CubeMapCameraModel)                                        \
  CAMERA_MODEL_CASE(DoubleSphereCameraModel)                                   \
  CAMERA_MODEL_CASE(ExtendedUnifiedCameraModel)                                \
  CAMERA_MODEL_CASE(UnifiedCameraModel)                                        \
  CAMERA_MODEL_CASE(FovCameraModel)                                            \
  CAMERA_MODEL_CASE(ScaramuzzaCameraModel)
#endif

#ifndef CAMERA_MODEL_SWITCH_CASES
#define CAMERA_MODEL_SWITCH_CASES                                              \
  CAMERA_MODEL_CASES                                                           \
  default:                                                                     \
    /*CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION*/                                  \
    break;
#endif

#define CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION                                  \
  throw std::domain_error("Camera model does not exist");

// class for all camera models.
class GeneralCameraModel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<GeneralCameraModel> Ptr;
  typedef std::shared_ptr<const GeneralCameraModel> ConstPtr;

public:
  GeneralCameraModel() {}
  ~GeneralCameraModel() {}

  void defaultInit();

  // Load and save camera model from/to file.
  bool loadConfigFile(const std::string &filename);
  void saveConfigFile(const std::string &filename) const;
  bool loadFromTxtFile(const std::string &filename);
  void saveToTxtFile(const std::string &filename) const;
  bool loadFromCerealJsonFile(const std::string &filename);
  void saveToCerealJsonFile(const std::string &filename) const;
  bool loadFromCerealBinaryFile(const std::string &filename);
  void saveToCerealBinaryFile(const std::string &filename) const;

  std::string info() const;

  // Access the camera name.
  std::string cameraName() const;
  void setCameraName(const std::string &camera_name);

  // Access the camera model.
  int modelId() const;
  std::string modelName() const;
  void setModelId(const int model_id);
  void setModelIdFromName(const std::string &model_name);

  // Access dimensions of the camera sensor.
  size_t width() const;
  size_t height() const;
  void setWidth(const size_t w);
  void setHeight(const size_t h);

  // Access the raw parameter vector.
  size_t numParams() const;
  const std::vector<double> &params() const;
  std::vector<double> &params();
  double params(const size_t idx) const;
  double &params(const size_t idx);
  const double *paramsData() const;
  double *paramsData();
  void setParams(const std::vector<double> &p);
  std::string paramsInfo() const;

  cv::Mat mask() { return camera_mask_.clone(); }
  bool hasMask() const { return !camera_mask_.empty(); }
  bool setMask(std::string mask_path);
  bool isPixelVaild(const Eigen::Matrix<double, 2, 1> &pixel) const;

  std::vector<double> getParams() const { return params_; }
  double getParamValue(const size_t idx) { 
    if (idx >= params_.size()) {
      throw std::out_of_range("Index out of range");
      return 0;
    }
    return params_[idx]; 
  }

  void cameraModelInitializeParams(const int model_id,
                                   const double focal_length,
                                   const size_t width, const size_t height);

  template <typename T>
  void spaceToPlane(const T *params, const Eigen::Matrix<T, 3, 1> &P,
                    Eigen::Matrix<T, 2, 1> *p,
                    Eigen::Matrix<T, 2, 3> *J = nullptr) const;

  template <typename T>
  void planeToSpace(const T *params, const Eigen::Matrix<T, 2, 1> &p,
                    Eigen::Matrix<T, 3, 1> *P) const;

  void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d *p,
                    Eigen::Matrix<double, 2, 3> *J = nullptr) const {
    spaceToPlane(params_.data(), P, p, J);
  }

  void planeToSpace(const Eigen::Vector2d &p, Eigen::Vector3d *P) const {
    planeToSpace(params_.data(), p, P);
  }

  Eigen::Vector2d spaceToPlane2(const Eigen::Vector3d &P) const {
    Eigen::Vector2d p;
    spaceToPlane(params_.data(), P, &p);
    return p;
  }

  Eigen::Vector3d planeToSpace2(const Eigen::Vector2d &p) const {
    Eigen::Vector3d P;
    planeToSpace(params_.data(), p, &P);
    return P;
  }

  bool getPixelValue(const cv::Mat &img, const Eigen::Vector2d &p,
                     cv::Vec3b &value,
                     const bool &is_linear_interp = true) const;

  // Undistort and rectify image.
  // with cv::remap
  void initUndistortRectifyMap(
      GeneralCameraModel new_camera = GeneralCameraModel(),
      const Eigen::Matrix3d &R = Eigen::Matrix3d::Identity(),
      cv::Mat *map1 = nullptr, cv::Mat *map2 = nullptr) const;

  template <class Archive> void serialize(Archive &ar) {
    ar(cereal::make_nvp("camera_name", camera_name_),
       cereal::make_nvp("model_id", model_id_),
       cereal::make_nvp("width", width_), cereal::make_nvp("height", height_),
       cereal::make_nvp("params", params_));
  }

  // only for ScaramuzzaCameraModel
  void generateExtraParams();

public:
  // Simple Pinhole camera model.
  //
  // No Distortion is assumed. Only focal length and principal point is modeled.
  //
  // Parameter list is expected in the following order:
  //
  //   f, cx, cy
  //
  // See https://en.wikipedia.org/wiki/Pinhole_camera_model
  CAMERA_MODEL_DEFINITIONS(SimplePinholeCameraModel, 0, "SIMPLE_PINHOLE", 3,
                           "f, cx, cy")

  // Equirectangular camera model for Pano images.
  //
  // Parameter list is expected in the following order:
  //
  //   w, h, r
  //
  CAMERA_MODEL_DEFINITIONS(EquirectangularCameraModel, 1, "EQUIRECTANGULAR", 3,
                           "w, h, r")

  // OpenCV camera model.
  //
  // Based on the pinhole camera model. Additionally models radial and
  // tangential distortion (up to 2nd degree of coefficients). Not suitable for
  // large radial distortions of fish-eye cameras.
  //
  // Parameter list is expected in the following order:
  //
  //    fx, fy, cx, cy, k1, k2, p1, p2
  //
  // See
  // http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  CAMERA_MODEL_DEFINITIONS(OpenCVPinholeCameraModel, 2, "OPENCV_PINHOLE", 8,
                           "fx, fy, cx, cy, k1, k2, p1, p2")

  // OpenCV fish-eye camera model.
  //
  // Based on the pinhole camera model. Additionally models radial and
  // tangential Distortion (up to 2nd degree of coefficients). Suitable for
  // large radial distortions of fish-eye cameras.
  //
  // Parameter list is expected in the following order:
  //
  //    fx, fy, cx, cy, k1, k2, k3, k4
  //
  // See
  // http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  CAMERA_MODEL_DEFINITIONS(OpenCVFisheyeCameraModel, 3, "OPENCV_FISHEYE", 8,
                           "fx, fy, cx, cy, k1, k2, k3, k4")

  // Omni fish-eye camera model.
  //
  // Based on the pinhole camera model. Additionally models radial and
  // tangential Distortion (up to 2nd degree of coefficients). Suitable for
  // large radial distortions of fish-eye cameras.
  //
  // Parameter list is expected in the following order:
  //
  //    xi, fx, fy, cx, cy, k1, k2, p1, p2
  //
  CAMERA_MODEL_DEFINITIONS(OmniDirectionalCameraModel, 4,
                           "OMNIDIRECTIONAL_CAMERA", 9,
                           "xi, fx, fy, cx, cy, k1, k2, p1, p2")

  // Camera model for CubeMap images.
  //
  // Parameter list is expected in the following order:
  //
  //   w
  //
  CAMERA_MODEL_DEFINITIONS(CubeMapCameraModel, 5, "CUBEMAP", 1, "w")

  // double sphere camera model.
  //
  // V. Usenko, N. Demmel, and D. Cremers, "The Double Sphere Camera Model",
  // Proc. of the Int. Conference on 3D Vision (3DV), 2018.
  //
  // Parameter list is expected in the following order:
  //
  //    fx, fy, cx, cy, xi, alpha
  //
  CAMERA_MODEL_DEFINITIONS(DoubleSphereCameraModel, 6, "DoubleSphere", 6,
                           "fx, fy, cx, cy, xi, alpha")

  // extended unified camera model
  //
  // Khomutenko, Bogdan, Gaëtan Garcia, and Philippe Martinet. "An enhanced
  // unified camera model." IEEE Robotics and Automation Letters 1.1 (2015):
  // 137-144.
  //
  // Parameter list is expected in the following order:
  //
  //   alpha, beta, fx, fy, cx, cy
  //
  CAMERA_MODEL_DEFINITIONS(ExtendedUnifiedCameraModel, 7, "EUCM", 6,
                           "alpha, beta, fx, fy, cx, cy")

  // extended unified camera model
  //
  // A generic fisheye camera model for robotic applications
  //
  // Parameter list is expected in the following order:
  //
  //   xi, fx, fy, cx, cy
  //
  CAMERA_MODEL_DEFINITIONS(UnifiedCameraModel, 8, "UCM", 5,
                           "xi, fx, fy, cx, cy")

  // fov camera model
  //
  // Frederic Devernay, Olivier Faugeras. Straight lines have to be straight:
  // Automatic calibration and removal of distortion from scenes of structured
  // environments. Machine vision and applications, 2001.
  //
  // Parameter list is expected in the following order:
  //
  //   omega, fx, fy, cx, cy
  //
  CAMERA_MODEL_DEFINITIONS(FovCameraModel, 9, "Fov", 5, "omega, fx, fy, cx, cy")

  // scaramuzza camera model
  //
  // https://sites.google.com/site/scarabotix/ocamcalib-toolbox
  //
  // Parameter list is expected in the following order:
  //
  //   c, d, e, cx, cy, poly_vec, inv_poly_vec
  //
  CAMERA_MODEL_DEFINITIONS(ScaramuzzaCameraModel, 10, "Scaramuzza",
                           SCARAMUZZA_PARAMS_SIZE,
                           "c, d, e, cx, cy, poly_vec, inv_poly_vec")

protected:
  std::string camera_name_ = "camera";
  int model_id_ = kInvalidCameraModelId;
  int width_ = 0;
  int height_ = 0;
  std::vector<double> params_;
  cv::Mat camera_mask_;
};

template <typename T>
void GeneralCameraModel::spaceToPlane(const T *params,
                                      const Eigen::Matrix<T, 3, 1> &P,
                                      Eigen::Matrix<T, 2, 1> *p,
                                      Eigen::Matrix<T, 2, 3> *J) const {
  switch (model_id_) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  case CameraModel::kModelId:                                                  \
    CameraModel::spaceToPlane(params, P, p, J);                                \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

template <typename T>
void GeneralCameraModel::planeToSpace(const T *params,
                                      const Eigen::Matrix<T, 2, 1> &p,
                                      Eigen::Matrix<T, 3, 1> *P) const {
  switch (model_id_) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  case CameraModel::kModelId:                                                  \
    CameraModel::planeToSpace(params, p, P);                                   \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

} // namespace general_camera_model

#include "general_camera_model/implementation/cubemap_camera_model.hpp"
#include "general_camera_model/implementation/double_sphere_camera_model.hpp"
#include "general_camera_model/implementation/equirectangular_camera_model.hpp"
#include "general_camera_model/implementation/extended_unified_camera_model.hpp"
#include "general_camera_model/implementation/fov_camera_model.hpp"
#include "general_camera_model/implementation/omni_directional_camera_model.hpp"
#include "general_camera_model/implementation/opencv_fisheye_camera_model.hpp"
#include "general_camera_model/implementation/opencv_pinhole_camera_model.hpp"
#include "general_camera_model/implementation/scaramuzza_camera_model.hpp"
#include "general_camera_model/implementation/simple_pinhole_camera_model.hpp"
#include "general_camera_model/implementation/unified_camera_model.hpp"