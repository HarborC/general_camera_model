namespace general_camera_model {

namespace omni_directional_functions {

template <typename T>
void distortion(const T *extra_params, const T u, const T v, T *du, T *dv) {
  T k1 = extra_params[0];
  T k2 = extra_params[1];
  T p1 = extra_params[2];
  T p2 = extra_params[3];

  T mx2_u = u * u;
  T my2_u = v * v;
  T mxy_u = u * v;
  T rho2_u = mx2_u + my2_u;
  T rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  *du = u * rad_dist_u + T(2.0) * p1 * mxy_u + p2 * (rho2_u + T(2.0) * mx2_u),
  *dv = v * rad_dist_u + T(2.0) * p2 * mxy_u + p1 * (rho2_u + T(2.0) * my2_u);
}

} // namespace omni_directional_functions

template <typename T>
void GeneralCameraModel::OmniDirectionalCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  const T xi = params[0];

  T len = sqrt(P(0) * P(0) + P(1) * P(1) + P(2) * P(2));
  T z = P(2) + xi * len;
  T u = P(0) / z;
  T v = P(1) / z;

  // Apply distortion
  T d_u, d_v;
  omni_directional_functions::distortion(&params[5], u, v, &d_u, &d_v);

  // Transform to image coordinates
  (*p)(0) = params[1] * (u + d_u) + params[3];
  (*p)(1) = params[2] * (v + d_v) + params[4];

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::OmniDirectionalCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  const T f1 = params[1];
  const T f2 = params[2];
  const T c1 = params[3];
  const T c2 = params[4];

  T x = p(0);
  T y = p(1);

  T mx_d, my_d, mx_u, my_u;

  // Lift points to normalised plane
  mx_d = (x - c1) / f1;
  my_d = (y - c2) / f2;

  // Recursive distortion model
  int n = 8;
  T d_x, d_y;
  omni_directional_functions::distortion(&params[5], mx_d, my_d, &d_x, &d_y);
  // Approximate value
  mx_u = mx_d - d_x;
  my_u = my_d - d_y;

  for (int i = 1; i < n; ++i) {
    omni_directional_functions::distortion(&params[5], mx_u, my_u, &d_x, &d_y);
    mx_u = mx_d - d_x;
    my_u = my_d - d_y;
  }

  // Obtain a projective ray
  T xi = params[0];
  if (ceres::abs(xi - 1.0) < T(1e-8)) {
    T lambda = T(2.0) / (mx_u * mx_u + my_u * my_u + T(1.0));
    (*P)(0) = lambda * mx_u;
    (*P)(1) = lambda * my_u;
    (*P)(2) = lambda - T(1.0);
  } else {
    T lambda = (xi + ceres::sqrt(T(1.0) + (T(1.0) - xi * xi) *
                                              (mx_u * mx_u + my_u * my_u))) /
               (T(1.0) + mx_u * mx_u + my_u * my_u);
    (*P)(0) = lambda * mx_u;
    (*P)(1) = lambda * my_u;
    (*P)(2) = lambda - xi;
  }
}

std::vector<double>
GeneralCameraModel::OmniDirectionalCameraModel::initializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {1.0, focal_length, focal_length, width / 2.0, height / 2.0, 0, 0, 0,
          0};
}

} // namespace general_camera_model