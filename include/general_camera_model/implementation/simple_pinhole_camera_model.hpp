namespace general_camera_model {

template <typename T>
void GeneralCameraModel::SimplePinholeCameraModel::spaceToPlane(
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

  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  T u = P(0) / P(2);
  T v = P(1) / P(2);

  // Transform to image coordinates
  (*p)(0) = f * u + c1;
  (*p)(1) = f * v + c2;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::SimplePinholeCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  T x = p(0);
  T y = p(1);

  (*P)(0) = (x - c1) / f;
  (*P)(1) = (y - c2) / f;
  (*P)(2) = 1.0;
}

std::vector<double>
GeneralCameraModel::SimplePinholeCameraModel::initializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0};
}

} // namespace general_camera_model