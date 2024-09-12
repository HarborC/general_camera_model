#include "general_camera_model/general_camera_model.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace general_camera_model {

// initialize
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  const int GeneralCameraModel::CameraModel::model_id =                        \
      GeneralCameraModel::CameraModel::modelId();                              \
  const std::string GeneralCameraModel::CameraModel::model_name =              \
      GeneralCameraModel::CameraModel::modelName();                            \
  const size_t GeneralCameraModel::CameraModel::num_params =                   \
      GeneralCameraModel::CameraModel::numParams();                            \
  const std::string GeneralCameraModel::CameraModel::params_info =             \
      GeneralCameraModel::CameraModel::paramsInfo();

CAMERA_MODEL_CASES

#undef CAMERA_MODEL_CASE

std::unordered_map<std::string, int> initializeCameraModelNameToId() {
  std::unordered_map<std::string, int> camera_model_name_to_id;

#define CAMERA_MODEL_CASE(CameraModel)                                         \
  camera_model_name_to_id.emplace(GeneralCameraModel::CameraModel::model_name, \
                                  GeneralCameraModel::CameraModel::model_id);

  CAMERA_MODEL_CASES

#undef CAMERA_MODEL_CASE

  return camera_model_name_to_id;
}

std::unordered_map<int, std::string> initializeCameraModelIdToName() {
  std::unordered_map<int, std::string> camera_model_id_to_name;

#define CAMERA_MODEL_CASE(CameraModel)                                         \
  camera_model_id_to_name.emplace(                                             \
      GeneralCameraModel::CameraModel::model_id,                               \
      GeneralCameraModel::CameraModel::model_name);

  CAMERA_MODEL_CASES

#undef CAMERA_MODEL_CASE

  return camera_model_id_to_name;
}

static const std::unordered_map<std::string, int> CAMERA_MODEL_NAME_TO_ID =
    initializeCameraModelNameToId();

static const std::unordered_map<int, std::string> CAMERA_MODEL_ID_TO_NAME =
    initializeCameraModelIdToName();

bool existsCameraModelWithName(const std::string &model_name) {
  return CAMERA_MODEL_NAME_TO_ID.count(model_name) > 0;
}

bool existsCameraModelWithId(const int model_id) {
  return CAMERA_MODEL_ID_TO_NAME.count(model_id) > 0;
}

int cameraModelNameToId(const std::string &model_name) {
  const auto it = CAMERA_MODEL_NAME_TO_ID.find(model_name);
  if (it == CAMERA_MODEL_NAME_TO_ID.end()) {
    return kInvalidCameraModelId;
  } else {
    return it->second;
  }
}

std::string cameraModelIdToName(const int model_id) {
  const auto it = CAMERA_MODEL_ID_TO_NAME.find(model_id);
  if (it == CAMERA_MODEL_ID_TO_NAME.end()) {
    return "";
  } else {
    return it->second;
  }
}

bool cameraModelVerifyParams(const int model_id,
                             const std::vector<double> &params) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  case GeneralCameraModel::CameraModel::kModelId:                              \
    if (params.size() == GeneralCameraModel::CameraModel::num_params) {        \
      return true;                                                             \
    }                                                                          \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }

  return false;
}

size_t cameraModelNumParams(const int model_id) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  case GeneralCameraModel::CameraModel::kModelId:                              \
    return GeneralCameraModel::CameraModel::num_params;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }

  return 0;
}

std::string cameraModelParamsInfo(const int model_id) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  case GeneralCameraModel::CameraModel::kModelId:                              \
    return GeneralCameraModel::CameraModel::params_info;                       \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }

  return "Camera model does not exist";
}

void GeneralCameraModel::defaultInit() {
  cameraModelInitializeParams(2, 240, 640, 480);
}

bool GeneralCameraModel::loadFromTxtFile(const std::string &filename) {
  if (filename == "") {
    return false;
  }

  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    return false;
  }

  std::string str;
  std::getline(ifs, str);
  if (str[0] != '#' || str == "") {
    if (str != "" && str[0] != '#') {
      setCameraName(filename);
      {
        std::stringstream ss;
        ss << str;
        ss >> model_id_ >> width_ >> height_;
        model_id_ -= 2;
        setModelId(model_id_);
      }

      {
        std::getline(ifs, str);
        for (int i = 0; i < str.size(); ++i) {
          if (str[i] == ',') {
            str[i] = ' ';
          }
        }
        std::stringstream ss;
        ss << str;
        for (size_t i = 0; i < params_.size(); ++i) {
          ss >> params_[i];
        }
      }

      return true;
    }

    return false;
  }

  std::getline(ifs, str);
  if (str[0] == '#' || str == "") {
    return false;
  } else {
    setCameraName(str);
  }

  std::getline(ifs, str);
  if (str[0] != '#' || str == "") {
    return false;
  }

  std::getline(ifs, str);
  if (str[0] == '#' || str == "") {
    return false;
  } else {
    std::stringstream ss;
    ss << str;
    ss >> model_id_ >> width_ >> height_;
    setModelId(model_id_);
  }

  std::getline(ifs, str);
  if (str[0] != '#' || str == "") {
    return false;
  }

  std::getline(ifs, str);
  if (str[0] == '#' || str == "") {
    return false;
  } else {
    std::stringstream ss;
    ss << str;
    for (size_t i = 0; i < params_.size(); ++i) {
      ss >> params_[i];
    }
  }

  ifs.close();

  return true;
}

bool GeneralCameraModel::loadConfigFile(const std::string &filename) {
  if (filename == "") {
    return false;
  }

  std::string ext = filename.substr(filename.find_last_of(".") + 1);
  if (ext == "json") {
    // 如果文件后缀是json，则使用json格式读取
    return loadFromCerealJsonFile(filename);
  } else if (ext == "txt") {
    // 如果文件后缀是txt，则使用txt格式读取
    return loadFromTxtFile(filename);
  } else {
    // 默认使用二进制格式读取
    return loadFromCerealBinaryFile(filename);
  }
}

void GeneralCameraModel::saveConfigFile(const std::string &filename) const {
  if (filename == "") {
    return;
  }

  if (filename.substr(filename.find_last_of(".") + 1) == "json") {
    // 如果文件后缀是json，则使用json格式保存
    saveToCerealJsonFile(filename);
  } else if (filename.substr(filename.find_last_of(".") + 1) == "txt") {
    // 如果文件后缀是txt，则使用txt格式保存
    saveToTxtFile(filename);
  } else {
    // 默认使用二进制格式保存
    saveToCerealBinaryFile(filename);
  }
}

void GeneralCameraModel::saveToTxtFile(const std::string &filename) const {
  std::ofstream ofs(filename);
  ofs << info();
  ofs.close();
}

bool GeneralCameraModel::loadFromCerealJsonFile(const std::string &filename) {
  std::ifstream is(filename);
  cereal::JSONInputArchive archive(is);
  archive(*this);

  return true;
}

void GeneralCameraModel::saveToCerealJsonFile(
    const std::string &filename) const {
  std::ofstream os(filename);
  cereal::JSONOutputArchive archive(os);
  archive(cereal::make_nvp("general_camera_model", *this));
}

bool GeneralCameraModel::loadFromCerealBinaryFile(const std::string &filename) {
  std::ifstream is(filename);
  cereal::BinaryInputArchive archive(is);
  archive(*this);

  return true;
}

void GeneralCameraModel::saveToCerealBinaryFile(
    const std::string &filename) const {
  std::ofstream os(filename, std::ios::binary);
  cereal::BinaryOutputArchive archive(os);
  archive(cereal::make_nvp("general_camera_model", *this));
}

std::string GeneralCameraModel::info() const {
  std::stringstream ss;
  ss << "# camera_name " << std::endl
     << cameraName() << std::endl
     << "# camera_model: " << modelName()
     << " | model_id camera_width camera_height" << std::endl;
  ss << model_id_ << " " << width_ << " " << height_ << std::endl;
  ss << paramsInfo();
  return ss.str();
}

int GeneralCameraModel::modelId() const { return model_id_; }

std::string GeneralCameraModel::modelName() const {
  return cameraModelIdToName(model_id_);
}

void GeneralCameraModel::setModelIdFromName(const std::string &model_name) {
  if (!existsCameraModelWithName(model_name)) {
    std::cerr << "no this camera model!" << std::endl;
    return;
  }
  model_id_ = cameraModelNameToId(model_name);
  params_.resize(cameraModelNumParams(model_id_), 0);
}

void GeneralCameraModel::setModelId(const int model_id) {
  if (!existsCameraModelWithId(model_id)) {
    std::cerr << "no this camera model!" << std::endl;
    return;
  }
  model_id_ = model_id;
  params_.resize(cameraModelNumParams(model_id_), 0);
}

std::string GeneralCameraModel::cameraName() const { return camera_name_; }

void GeneralCameraModel::setCameraName(const std::string &camera_name) {
  camera_name_ = camera_name;
}

size_t GeneralCameraModel::width() const { return width_; }

size_t GeneralCameraModel::height() const { return height_; }

void GeneralCameraModel::setWidth(const size_t w) { width_ = w; }

void GeneralCameraModel::setHeight(const size_t h) { height_ = h; }

size_t GeneralCameraModel::numParams() const { return params_.size(); }

const std::vector<double> &GeneralCameraModel::params() const {
  return params_;
}

std::vector<double> &GeneralCameraModel::params() { return params_; }

double GeneralCameraModel::params(const size_t idx) const {
  return params_[idx];
}

double &GeneralCameraModel::params(const size_t idx) { return params_[idx]; }

const double *GeneralCameraModel::paramsData() const { return params_.data(); }

double *GeneralCameraModel::paramsData() { return params_.data(); }

void GeneralCameraModel::setParams(const std::vector<double> &p) {
  params_ = p;
}

std::string GeneralCameraModel::paramsInfo() const {
  std::stringstream ss;
  ss << "# params list : " << cameraModelParamsInfo(model_id_) << std::endl;
  for (size_t i = 0; i < params_.size(); ++i) {
    ss << std::fixed << std::setprecision(6) << params_[i] << " ";
  }
  ss << std::endl;

  return ss.str();
}

void GeneralCameraModel::cameraModelInitializeParams(const int model_id,
                                                     const double focal_length,
                                                     const size_t width,
                                                     const size_t height) {
  // Assuming that image measurements are within [0, dim], i.e. that the
  // upper left corner is the (0, 0) coordinate (rather than the center of
  // the upper left pixel). This complies with the default SiftGPU convention.
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  case CameraModel::kModelId:                                                  \
    model_id_ = model_id;                                                      \
    width_ = width;                                                            \
    height_ = height;                                                          \
    params_ = CameraModel::initializeParams(focal_length, width, height);      \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

bool GeneralCameraModel::getPixelValue(const cv::Mat &img,
                                       const Eigen::Vector2d &p,
                                       cv::Vec3b &value,
                                       const bool &is_linear_interp) const {
  int w = img.cols;
  int h = img.rows;

  assert(w == width_ && h == height_);

  if (p(0) < 0 || p(0) > w - 1 || p(1) < 0 || p(1) > h - 1)
    return false;

  if (!is_linear_interp) {
    int x = int(p(0) + 0.5);
    int y = int(p(1) + 0.5);
    value = img.at<cv::Vec3b>(y, x);
  } else {
    float x = float(p(0));
    float y = float(p(1));
    int x1 = int(x);
    int y1 = int(y);
    int x2 = x1 + 1;
    int y2 = y1 + 1;

    value = (x - x1) * (y - y1) * img.at<cv::Vec3b>(y2, x2) +
            (x2 - x) * (y - y1) * img.at<cv::Vec3b>(y2, x1) +
            (x - x1) * (y2 - y) * img.at<cv::Vec3b>(y1, x2) +
            (x2 - x) * (y2 - y) * img.at<cv::Vec3b>(y1, x1);
  }

  return true;
}

void GeneralCameraModel::initUndistortRectifyMap(GeneralCameraModel new_camera,
                                                 const Eigen::Matrix3d &R,
                                                 cv::Mat *map1,
                                                 cv::Mat *map2) const {
  if (new_camera.modelId() < 0) {
    new_camera.cameraModelInitializeParams(0, std::max(width(), height()) / 2,
                                           width(), height());
  }

  int h = new_camera.height();
  int w = new_camera.width();

  if (map1)
    *map1 = cv::Mat::zeros(h, w, CV_32FC1);
  if (map2)
    *map2 = cv::Mat::zeros(h, w, CV_32FC1);

  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      Eigen::Vector2d p0(u, v);
      Eigen::Vector3d P0;
      new_camera.planeToSpace(p0, &P0);

      if (P0.norm() < 1e-6)
        continue;

      Eigen::Vector3d P1 = R * P0;
      Eigen::Vector2d p1;
      spaceToPlane(P1, &p1);

      if (map1)
        map1->at<float>(v, u) = float(p1(0));
      if (map2)
        map2->at<float>(v, u) = float(p1(1));
    }
  }
}

void GeneralCameraModel::generateExtraParams() {
  // ScaramuzzaCameraModel
  if (model_id_ == 10) {
    auto all_param =
        scaramuzza_functions::generateAllParam(width_, height_, params_);
    params_ = all_param;
  }
}

bool GeneralCameraModel::setMask(std::string mask_path) {
  cv::Mat mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
  if (mask.empty()) {
    std::cerr << "Failed to read mask image: " << mask_path << std::endl;
    return false;
  }

  camera_mask_ = mask.clone();
  return true;
}

bool GeneralCameraModel::isPixelVaild(
    const Eigen::Matrix<double, 2, 1> &pixel) const {
  if (pixel(0) < 0 || pixel(0) >= width_ || pixel(1) < 0 || pixel(1) >= height_) {
    return false;
  }
  if (!camera_mask_.empty() &&
      camera_mask_.at<uchar>(pixel(1), pixel(0)) < 128) {
    return false;
  }
  return true;
}

} // namespace general_camera_model