namespace general_camera_model {

template <typename T>
void GeneralCameraModel::UnifiedCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  const T &xi = params[0];
  const T &fx = params[1];
  const T &fy = params[2];
  const T &cx = params[3];
  const T &cy = params[4];

  T rho = ceres::sqrt(P(2) * P(2) + P(0) * P(0) + P(1) * P(1));
  T denominv = T(1.) / (P(2) + xi * rho);

  // Project the point to the mu plane
  T xn = P(0) * denominv;
  T yn = P(1) * denominv;

  // Compute image point
  (*p)(0) = fx * xn + cx;
  (*p)(1) = fy * yn + cy;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::UnifiedCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  const T &xi = params[0];
  const T &fx = params[1];
  const T &fy = params[2];
  const T &cx = params[3];
  const T &cy = params[4];

  T xn = (p(0) - cx) / fx;
  T yn = (p(1) - cy) / fy;

  T u2 = xn * xn + yn * yn;

  T gamma = ceres::sqrt(T(1.) + u2 * (T(1) - xi * xi));

  T etanum = -gamma - xi * u2;
  T etadenom = xi * xi * u2 - T(1);

  (*P)(0) = xn;
  (*P)(1) = yn;
  (*P)(2) = etadenom / (etadenom + xi * etanum);
}

std::vector<double> GeneralCameraModel::UnifiedCameraModel::initializeParams(
    double focal_length, const size_t width, const size_t height) {
  return {1.0, focal_length, focal_length, width / 2.0, height / 2.0};
}

} // namespace general_camera_model