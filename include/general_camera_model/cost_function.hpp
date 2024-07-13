#pragma once
#include <ceres/ceres.h>

namespace general_camera_model {

class CameraModelConvertCostFunction {
public:
  explicit CameraModelConvertCostFunction(const Eigen::Vector2d &point2D,
                                          const Eigen::Vector3d &point3D,
                                          GeneralCameraModel::Ptr &camera)
      : observed_x_(point2D(0)), observed_y_(point2D(1)), cpt_(point3D),
        camera_(camera) {}

  static auto *Create(const Eigen::Vector2d &point2D,
                      const Eigen::Vector3d &point3D,
                      GeneralCameraModel::Ptr &camera) {
    using Functor = CameraModelConvertCostFunction;
    auto *cost_function = new ceres::DynamicNumericDiffCostFunction<Functor>(
        new Functor(point2D, point3D, camera));

    cost_function->AddParameterBlock(camera->numParams());
    cost_function->SetNumResiduals(2);

    return cost_function;
  }

  void test(double const *const *parameters, double *residuals) {
    Eigen::Matrix<double, 3, 1> pc = cpt_.template cast<double>();

    // Distort and transform to pixel space.
    Eigen::Matrix<double, 2, 1> pc_distorted;
    camera_->spaceToPlane(parameters[0], pc, &pc_distorted);

    // Re-projection error.
    residuals[0] = pc_distorted(0) - double(observed_x_);
    residuals[1] = pc_distorted(1) - double(observed_y_);

    if (residuals[0] == 0 && residuals[1] == 0) {
      std::cout << "pc_distorted: " << pc_distorted.transpose() << std::endl;
      std::cout << "residuals: " << residuals[0] << " " << residuals[1] << std::endl;
    }
    
    if (std::isnan(residuals[0]) || std::isnan(residuals[1])) {
        std::cout << "pc_distorted: " << pc_distorted.transpose() << std::endl;
      std::cout << "residuals: " << residuals[0] << " " << residuals[1] << std::endl;
    }
  }

  bool operator()(double const *const *parameters, double *residuals) const {
    Eigen::Matrix<double, 3, 1> pc = cpt_;

    // Distort and transform to pixel space.
    Eigen::Matrix<double, 2, 1> pc_distorted;
    camera_->spaceToPlane(parameters[0], pc, &pc_distorted);

    // Re-projection error.
    residuals[0] = pc_distorted(0) - double(observed_x_);
    residuals[1] = pc_distorted(1) - double(observed_y_);

    return true;
  }

public:
  const double observed_x_;
  const double observed_y_;
  Eigen::Vector3d cpt_;
  GeneralCameraModel::Ptr camera_;
};

class BundleAdjustmentFixPointCostFunction {
public:
  explicit BundleAdjustmentFixPointCostFunction(const Eigen::Vector2d &point2D,
                                                const Eigen::Vector3d &point3D,
                                                GeneralCameraModel::Ptr &camera)
      : observed_x_(point2D(0)), observed_y_(point2D(1)), cpt_(point3D),
        camera_(camera) {}

  static auto *Create(const Eigen::Vector2d &point2D,
                      const Eigen::Vector3d &point3D,
                      GeneralCameraModel::Ptr &camera) {
    using Functor = BundleAdjustmentFixPointCostFunction;
    auto *cost_function = new ceres::DynamicAutoDiffCostFunction<Functor>(
        new Functor(point2D, point3D, camera));

    cost_function->AddParameterBlock(4);
    cost_function->AddParameterBlock(3);
    cost_function->AddParameterBlock(camera->numParams());

    cost_function->SetNumResiduals(2);

    return cost_function;
  }

  // void test(double const *const *parameters, double *residuals) {
  //   Eigen::Quaternion<double> qcw(parameters[0][3], parameters[0][0],
  //                            parameters[0][1], parameters[0][2]);
  //   Eigen::Matrix<double, 3, 1> tcw(parameters[1][0], parameters[1][1],
  //                              parameters[1][2]);
  //   Eigen::Matrix<double, 3, 1> pw = cpt_.template cast<double>();
  //   std::cout << "pw: " << pw.transpose() << std::endl;

  //   Eigen::Matrix<double, 3, 1> pc = qcw * pw + tcw;
  //   std::cout << "pc: " << pc.transpose() << std::endl;

  //   // Distort and transform to pixel space.
  //   Eigen::Matrix<double, 2, 1> pc_distorted;
  //   camera_.spaceToPlane(parameters[2], pc, &pc_distorted);
  //   std::cout << "pc_distorted: " << pc_distorted.transpose() << std::endl;

  //   // Re-projection error.
  //   residuals[0] = pc_distorted(0) - double(observed_x_);
  //   residuals[1] = pc_distorted(1) - double(observed_y_);
  // }

  template <typename T>
  bool operator()(T const *const *parameters, T *residuals) const {
    Eigen::Quaternion<T> qcw(parameters[0][3], parameters[0][0],
                             parameters[0][1], parameters[0][2]);
    Eigen::Matrix<T, 3, 1> tcw(parameters[1][0], parameters[1][1],
                               parameters[1][2]);
    Eigen::Matrix<T, 3, 1> pw = cpt_.template cast<T>();

    Eigen::Matrix<T, 3, 1> pc = qcw * pw + tcw;

    // Distort and transform to pixel space.
    Eigen::Matrix<T, 2, 1> pc_distorted;
    camera_->spaceToPlane(parameters[2], pc, &pc_distorted);

    // Re-projection error.
    residuals[0] = pc_distorted(0) - T(observed_x_);
    residuals[1] = pc_distorted(1) - T(observed_y_);

    return true;
  }

public:
  const double observed_x_;
  const double observed_y_;
  Eigen::Vector3d cpt_;
  GeneralCameraModel::Ptr camera_;
};

} // namespace general_camera_model