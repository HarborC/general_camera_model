namespace general_camera_model {

namespace opencv_pinhole_functions {
template <typename T>
inline void distortion(const T *extra_params, const T u, const T v, T *du,
                       T *dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k1 * r2 + k2 * r2 * r2;
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2);
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2);
}

template <typename T>
inline void iterativeUndistortion(const T *params, T *u, T *v) {
  const size_t kNumIterations = 100;
  const double kMaxStepNorm = 1e-10;
  const double kRelStepSize = 1e-6;

  Eigen::Matrix2d J;
  const Eigen::Vector2d x0(*u, *v);
  Eigen::Vector2d x(*u, *v);
  Eigen::Vector2d dx;
  Eigen::Vector2d dx_0b;
  Eigen::Vector2d dx_0f;
  Eigen::Vector2d dx_1b;
  Eigen::Vector2d dx_1f;

  for (size_t i = 0; i < kNumIterations; ++i) {
    const double step0 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(0)));
    const double step1 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(1)));
    distortion(params, x(0), x(1), &dx(0), &dx(1));
    distortion(params, x(0) - step0, x(1), &dx_0b(0), &dx_0b(1));
    distortion(params, x(0) + step0, x(1), &dx_0f(0), &dx_0f(1));
    distortion(params, x(0), x(1) - step1, &dx_1b(0), &dx_1b(1));
    distortion(params, x(0), x(1) + step1, &dx_1f(0), &dx_1f(1));
    J(0, 0) = 1 + (dx_0f(0) - dx_0b(0)) / (2 * step0);
    J(0, 1) = (dx_1f(0) - dx_1b(0)) / (2 * step1);
    J(1, 0) = (dx_0f(1) - dx_0b(1)) / (2 * step0);
    J(1, 1) = 1 + (dx_1f(1) - dx_1b(1)) / (2 * step1);
    const Eigen::Vector2d step_x = J.inverse() * (x + dx - x0);
    x -= step_x;
    if (step_x.squaredNorm() < kMaxStepNorm) {
      break;
    }
  }

  *u = x(0);
  *v = x(1);
}
} // namespace opencv_pinhole_functions

template <typename T>
void GeneralCameraModel::OpenCVPinholeCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  if (P(2) < 1e-6) {
    (*p)(0) = T(-1);
    (*p)(1) = T(-1);
    return;
  }

  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  T u = P(0) / P(2);
  T v = P(1) / P(2);

  // Distortion
  T du = T(0.0);
  T dv = T(0.0);
  if (ceres::abs(params[4] - T(0.0)) > T(1e-8))
    opencv_pinhole_functions::distortion(&params[4], u, v, &du, &dv);
  (*p)(0) = u + du;
  (*p)(1) = v + dv;

  // Transform to image coordinates
  (*p)(0) = f1 * (*p)(0) + c1;
  (*p)(1) = f2 * (*p)(1) + c2;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::OpenCVPinholeCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  T x = p(0);
  T y = p(1);

  // Lift points to normalized plane
  T u = (x - c1) / f1;
  T v = (y - c2) / f2;

  if (ceres::abs(params[4] - T(0.0)) > T(1e-8))
    opencv_pinhole_functions::iterativeUndistortion(&params[4], &u, &v);

  (*P)(0) = u;
  (*P)(1) = v;
  (*P)(2) = 1.0;
}

std::vector<double>
GeneralCameraModel::OpenCVPinholeCameraModel::initializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 0, 0, 0, 0};
}

} // namespace general_camera_model