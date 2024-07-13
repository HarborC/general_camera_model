namespace general_camera_model {

template <typename T>
void GeneralCameraModel::EquirectangularCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  const T &width = params[0];
  const T &height = params[1];

  Eigen::Matrix<T, 3, 1> bearing = P.normalized();
  // convert to unit polar coordinates
  const T lat = -ceres::asin(bearing(1));
  const T lon = ceres::atan2(bearing(0), bearing(2));
  // convert to pixel image coordinated
  *p = Eigen::Matrix<T, 2, 1>((width - T(1)) * (T(0.5) + lon / T(2.0 * M_PI)),
                              (height - T(1)) * (T(0.5) - lat / T(M_PI)));

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::EquirectangularCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  const T &width = params[0];
  const T &height = params[1];

  // 将全景影像的平面坐标转换到相机坐标系下
  const T lon = (p(0) / (width - T(1)) - T(0.5)) * T(2 * M_PI);
  const T lat = -(p(1) / (height - T(1)) - T(0.5)) * T(M_PI);
  // convert to equirectangular coordinates
  (*P)(0) = ceres::cos(lat) * ceres::sin(lon);
  (*P)(1) = -ceres::sin(lat);
  (*P)(2) = ceres::cos(lat) * ceres::cos(lon);
}

std::vector<double>
GeneralCameraModel::EquirectangularCameraModel::initializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {width, height, focal_length};
}

} // namespace general_camera_model