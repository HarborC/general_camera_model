namespace general_camera_model {

template <typename T>
void GeneralCameraModel::DoubleSphereCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  T fx = params[0];
  T fy = params[1];
  T cx = params[2];
  T cy = params[3];
  T xi = params[4];
  T alpha = params[5];

  T xx = P(0) * P(0);
  T yy = P(1) * P(1);
  T zz = P(2) * P(2);

  T r2 = xx + yy;
  T d1_2 = r2 + zz;
  T d1 = ceres::sqrt(d1_2);

  T w1 = alpha > T(0.5) ? (T(1) - alpha) / alpha : alpha / (T(1) - alpha);
  T w2 = (w1 + xi) / ceres::sqrt(T(2) * w1 * xi + xi * xi + T(1));
  if (P(2) <= -w2 * d1)
    return;

  T k = xi * d1 + P(2);
  T kk = k * k;

  T d2_2 = r2 + kk;
  T d2 = ceres::sqrt(d2_2);

  T norm = alpha * d2 + (T(1) - alpha) * k;

  T mx = P(0) / norm;
  T my = P(1) / norm;

  (*p)(0) = fx * mx + cx;
  (*p)(1) = fy * my + cy;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::DoubleSphereCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  T fx = params[0];
  T fy = params[1];
  T cx = params[2];
  T cy = params[3];
  T xi = params[4];
  T alpha = params[5];

  T mx = (p(0) - cx) / fx;
  T my = (p(1) - cy) / fy;

  T r2 = mx * mx + my * my;

  if (alpha > T(0.5)) {
    if (r2 >= T(1) / (T(2) * alpha - T(1)))
      return;
  }

  T xi2_2 = alpha * alpha;
  T xi1_2 = xi * xi;

  T sqrt2 = ceres::sqrt(T(1) - (T(2) * alpha - T(1)) * r2);

  T norm2 = alpha * sqrt2 + T(1) - alpha;

  T mz = (T(1) - xi2_2 * r2) / norm2;
  T mz2 = mz * mz;

  T norm1 = mz2 + r2;
  T sqrt1 = ceres::sqrt(mz2 + (T(1) - xi1_2) * r2);
  T k = (mz * xi + sqrt1) / norm1;

  (*P)(0) = k * mx;
  (*P)(1) = k * my;
  (*P)(2) = k * mz - xi;
}

std::vector<double>
GeneralCameraModel::DoubleSphereCameraModel::initializeParams(
    double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 1.0, 0};
}

} // namespace general_camera_model