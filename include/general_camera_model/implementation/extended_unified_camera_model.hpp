namespace general_camera_model {

template <typename T>
void GeneralCameraModel::ExtendedUnifiedCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  const T &alpha = params[0];
  const T &beta = params[1];
  const T &fx = params[2];
  const T &fy = params[3];
  const T &cx = params[4];
  const T &cy = params[5];

  T denom =
      alpha * ceres::sqrt(P(2) * P(2) + beta * (P(0) * P(0) + P(1) * P(1))) +
      (T(1.) - alpha) * P(2);
  if (denom < T(1e-3))
    return;

  // Check that the point is in the upper hemisphere in case of ellipsoid
  if (alpha > T(0.5)) {
    const T zn = P(2) / denom;
    const T C = (alpha - T(1.)) / (alpha + alpha - T(1.));
    if (zn < C)
      return;
  }

  // Project the point to the mu plane
  const T xn = P(0) / denom;
  const T yn = P(1) / denom;
  // Compute image point
  (*p)(0) = fx * xn + cx;
  (*p)(1) = fy * yn + cy;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::ExtendedUnifiedCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  const T &alpha = params[0];
  const T &beta = params[1];
  const T &fx = params[2];
  const T &fy = params[3];
  const T &cx = params[4];
  const T &cy = params[5];

  T xn = (p(0) - cx) / fx;
  T yn = (p(1) - cy) / fy;

  T u2 = xn * xn + yn * yn;
  T gamma = T(1.) - alpha;
  T num = T(1.) - u2 * alpha * alpha * beta;
  T det = T(1.) - (alpha - gamma) * beta * u2;
  if (det < 0)
    return;
  T denom = gamma + alpha * ceres::sqrt(det);

  (*P)(0) = xn;
  (*P)(1) = yn;
  (*P)(2) = num / denom;
}

std::vector<double>
GeneralCameraModel::ExtendedUnifiedCameraModel::initializeParams(
    double focal_length, const size_t width, const size_t height) {
  return {0, 0, focal_length, focal_length, width / 2.0, height / 2.0};
}

} // namespace general_camera_model