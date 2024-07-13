namespace general_camera_model {

namespace cubemap_functions {

template <typename T>
void function(const T *extra_params, const T &u_, const T &v_, T *x, T *y,
              T *z) {
  T w = extra_params[0];
  T f = w / T(2.0);

  T u = u_ + T(0.5);
  T v = v_ + T(0.5);

  int i0 = int(u / w);
  int i1 = int(v / w);

  // -----------------------------
  // |      |  10  |      |      |
  // -----------------------------
  // |  01  |  11  |  21  |  31  |
  // -----------------------------
  // |      |  12  |      |      |
  // -----------------------------

  if (i1 == 2 && i0 != 1 && (v - w * T(2.0) < T(1e-9))) {
    i1 = 1;
  }

  if (i0 == 1 && i1 == 1) {
    T new_u = u - w * T(1.5);
    T new_v = v - w * T(1.5);

    *x = new_u / f;
    *y = new_v / f;
    *z = T(1.0);
  } else if (i0 == 0 && i1 == 1) {
    T new_u = u - w * T(0.5);
    T new_v = v - w * T(1.5);

    *x = T(-1.0);
    *y = new_v / f;
    *z = new_u / f;
  } else if (i0 == 1 && i1 == 0) {
    T new_u = u - w * T(1.5);
    T new_v = v - w * T(0.5);

    *x = new_u / f;
    *y = T(-1.0);
    *z = new_v / f;
  } else if (i0 == 1 && i1 == 2) {
    T new_u = u - w * T(1.5);
    T new_v = v - w * T(2.5);

    *x = new_u / f;
    *y = T(1.0);
    *z = -new_v / f;
  } else if (i0 == 2 && i1 == 1) {
    T new_u = u - w * T(2.5);
    T new_v = v - w * T(1.5);

    *x = T(1.0);
    *y = new_v / f;
    *z = -new_u / f;
  } else if (i0 == 3 && i1 == 1) {
    T new_u = u - w * T(3.5);
    T new_v = v - w * T(1.5);

    *x = -new_u / f;
    *y = new_v / f;
    *z = T(-1.0);
  } else {
    *x = T(0.0);
    *y = T(0.0);
    *z = T(0.0);
  }
}

template <typename T>
void function2(const T *extra_params, const T &x, const T &y, const T &z, T *u,
               T *v) {
  T w = extra_params[0];
  T f = w / T(2.0);

  T abs_x = ceres::abs(x);
  T abs_y = ceres::abs(y);
  T abs_z = ceres::abs(z);

  if (abs_x >= abs_y && abs_x >= abs_z) {
    if (x > T(0.0)) {
      *u = -f * (z / x) + w * T(2.5) - T(0.5);
      *v = f * (y / x) + w * T(1.5) - T(0.5);
      // printf("%d%d\n", 2, 1);
    } else {
      *u = -f * (z / x) + w * T(0.5) - T(0.5);
      *v = -f * (y / x) + w * T(1.5) - T(0.5);
      // printf("%d%d\n", 0, 1);
    }
  } else if (abs_y >= abs_x && abs_y >= abs_z) {
    if (y > T(0.0)) {
      *u = f * (x / y) + w * T(1.5) - T(0.5);
      *v = -f * (z / y) + w * T(2.5) - T(0.5);
      // printf("%d%d\n", 1, 2);
    } else {
      *u = -f * (x / y) + w * T(1.5) - T(0.5);
      *v = -f * (z / y) + w * T(0.5) - T(0.5);
      // printf("%d%d\n", 1, 0);
    }
  } else if (abs_z >= abs_x && abs_z >= abs_y) {
    if (z > T(0.0)) {
      *u = f * (x / z) + w * T(1.5) - T(0.5);
      *v = f * (y / z) + w * T(1.5) - T(0.5);
      // printf("%d%d\n", 1, 1);
    } else {
      *u = -f * (x / z) + w * T(3.5) - T(0.5);
      *v = -f * (y / z) + w * T(1.5) - T(0.5);
      // printf("%d%d\n", 3, 1);
    }
  } else {
    *u = T(0.0);
    *v = T(0.0);
  }
}

} // namespace cubemap_functions

template <typename T>
void GeneralCameraModel::CubeMapCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  T u, v;
  cubemap_functions::function2(&(params[0]), P(0), P(1), P(2), &u, &v);

  // Transform to image coordinates
  (*p)(0) = u;
  (*p)(1) = v;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::CubeMapCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  T x, y, z;
  cubemap_functions::function(&(params[0]), p(0), p(1), &x, &y, &z);
  T norm = ceres::sqrt(x * x + y * y + z * z);

  (*P)(0) = x;
  (*P)(1) = y;
  (*P)(2) = z;

  if (norm < T(1e-6)) {
    // printf("norm is too small\n");
    return;
  }

  (*P)(0) = x / norm;
  (*P)(1) = y / norm;
  (*P)(2) = z / norm;
}

std::vector<double> GeneralCameraModel::CubeMapCameraModel::initializeParams(
    const double focal_length, const size_t width, const size_t height) {
  assert(8 * focal_length - width < 1e-6);
  assert(6 * focal_length - height < 1e-6);
  return {focal_length * 2};
}

} // namespace general_camera_model