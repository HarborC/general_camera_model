namespace general_camera_model {

namespace fov_functions {

template <typename T> T rd(T omg, T ru) {
  return T(1) / (omg)*ceres::atan(T(2) * ru * ceres::tan(omg * 0.5));
}

template <typename T> T rd_inverse(T omg, T rd) {
  return ceres::tan(rd * omg) / (T(2) * ceres::tan(omg * 0.5));
}

template <typename T> T ru(T x, T y, T z) {
  return ceres::sqrt((x * x + y * y) / (z * z));
}

} // namespace fov_functions

template <typename T>
void GeneralCameraModel::FovCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  const T &omega = params[0];
  const T &fx = params[1];
  const T &fy = params[2];
  const T &cx = params[3];
  const T &cy = params[4];

  T ru_p = fov_functions::ru(P(0), P(1), P(2));
  T rd_p = fov_functions::rd(omega, ru_p);

  // Compute image point
  (*p)(0) = fx * (P(0) / P(2) * rd_p / ru_p) + cx;
  (*p)(1) = fy * (P(1) / P(2) * rd_p / ru_p) + cy;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::FovCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  const T &omega = params[0];
  const T &fx = params[1];
  const T &fy = params[2];
  const T &cx = params[3];
  const T &cy = params[4];

  Eigen::Matrix<T, 2, 1> p_u(p(0) - cx, p(1) - cy);
  T rd = p_u.norm() / fx;

  T ru_p = fov_functions::rd_inverse(omega, rd);

  Eigen::Matrix<T, 3, 1> P_temp = Eigen::Matrix<T, 3, 1>(
      (p_u(0)) * ru_p / rd / fx, (p_u(1)) * ru_p / rd / fy, 1.0);
  T norm = P_temp.norm();

  (*P)(0) = P_temp(0) / norm;
  (*P)(1) = P_temp(1) / norm;
  (*P)(2) = P_temp(2) / norm;
}

std::vector<double> GeneralCameraModel::FovCameraModel::initializeParams(
    double focal_length, const size_t width, const size_t height) {
  return {1.0, focal_length, focal_length, width / 2.0, height / 2.0};
}

} // namespace general_camera_model