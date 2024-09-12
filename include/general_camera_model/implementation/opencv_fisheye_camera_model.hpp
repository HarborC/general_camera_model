#include <complex>

namespace general_camera_model {

namespace opencv_fisheye_functions {

template <typename T>
void distortion(const T *extra_params, const T u, const T v, T *du, T *dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T k3 = extra_params[2];
  const T k4 = extra_params[3];

  const T r = ceres::sqrt(u * u + v * v);

  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    const T theta8 = theta4 * theta4;
    const T thetad =
        theta * (T(1) + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    *du = u * thetad / r - u;
    *dv = v * thetad / r - v;
  } else {
    *du = T(0);
    *dv = T(0);
  }
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

template <typename T>
inline void backprojectSymmetric(const T *extra_params,
                                 const Eigen::Matrix<T, 2, 1> &p_u, T &theta,
                                 T &phi) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T k3 = extra_params[2];
  const T k4 = extra_params[3];

  T p_u_norm = p_u.norm();

  if (p_u_norm < T(1e-10)) {
    phi = T(0.0);
  } else {
    phi = ceres::atan2(p_u(1), p_u(0));
  }

  int npow = 9;
  if (k4 == T(0.0)) {
    npow -= 2;
  }
  if (k3 == T(0.0)) {
    npow -= 2;
  }
  if (k2 == T(0.0)) {
    npow -= 2;
  }
  if (k1 == T(0.0)) {
    npow -= 2;
  }

  Eigen::Matrix<T, Eigen::Dynamic, 1> coeffs(npow + 1);
  coeffs.setZero();
  coeffs(0) = -p_u_norm;
  coeffs(1) = T(1.0);

  if (npow >= 3) {
    coeffs(3) = k1;
  }
  if (npow >= 5) {
    coeffs(5) = k2;
  }
  if (npow >= 7) {
    coeffs(7) = k3;
  }
  if (npow >= 9) {
    coeffs(9) = k4;
  }

  if (npow == 1) {
    theta = p_u_norm;
  } else {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A(npow, npow);
    A.setZero();
    A.block(1, 0, npow - 1, npow - 1).setIdentity();
    A.col(npow - 1) = -coeffs.block(0, 0, npow, 1) / coeffs(npow);

    Eigen::EigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> es(A);
    Eigen::Matrix<std::complex<T>, Eigen::Dynamic, Eigen::Dynamic> eigval =
        es.eigenvalues();

    std::vector<T> thetas;
    for (int i = 0; i < eigval.rows(); ++i) {
      if (ceres::abs(eigval(i).imag()) > T(1e-10)) {
        continue;
      }

      T t = eigval(i).real();

      if (t < -T(1e-10)) {
        continue;
      } else if (t < T(0.0)) {
        t = T(0.0);
      }

      thetas.push_back(t);
    }

    if (thetas.empty()) {
      theta = p_u_norm;
    } else {
      theta = *std::min_element(thetas.begin(), thetas.end());
    }
  }
}
} // namespace opencv_fisheye_functions

template <typename T>
void GeneralCameraModel::OpenCVFisheyeCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  T theta = ceres::acos(P(2) / P.norm());
  T phi = ceres::atan2(P(1), P(0));

  const T k1 = params[4];
  const T k2 = params[5];
  const T k3 = params[6];
  const T k4 = params[7];

  const T theta2 = theta * theta;
  const T theta4 = theta2 * theta2;
  const T theta6 = theta4 * theta2;
  const T theta8 = theta4 * theta4;
  const T thetad =
      theta * (T(1) + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

  (*p)(0) = thetad * ceres::cos(phi);
  (*p)(1) = thetad * ceres::sin(phi);

  // Transform to image coordinates
  (*p)(0) = f1 * (*p)(0) + c1;
  (*p)(1) = f2 * (*p)(1) + c2;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::OpenCVFisheyeCameraModel::planeToSpace(
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
  Eigen::Matrix<T, 2, 1> p_u((x - c1) / f1, (y - c2) / f2);

  T theta, phi;
  opencv_fisheye_functions::backprojectSymmetric(&params[4], p_u, theta, phi);

  (*P)(0) = ceres::sin(theta) * ceres::cos(phi);
  (*P)(1) = ceres::sin(theta) * ceres::sin(phi);
  (*P)(2) = ceres::cos(theta);
}

std::vector<double>
GeneralCameraModel::OpenCVFisheyeCameraModel::initializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 0, 0, 0, 0};
}

} // namespace general_camera_model