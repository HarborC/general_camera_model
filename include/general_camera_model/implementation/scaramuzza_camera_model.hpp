namespace general_camera_model {

namespace scaramuzza_functions {

inline Eigen::VectorXd polyfit(Eigen::VectorXd &xVec, Eigen::VectorXd &yVec,
                               int poly_order) {
  assert(poly_order > 0);
  assert(xVec.size() > poly_order);
  assert(xVec.size() == yVec.size());

  Eigen::MatrixXd A(xVec.size(), poly_order + 1);
  Eigen::VectorXd B(xVec.size());

  for (int i = 0; i < xVec.size(); ++i) {
    const double x = xVec(i);
    const double y = yVec(i);

    double x_pow_k = 1.0;

    for (int k = 0; k <= poly_order; ++k) {
      A(i, k) = x_pow_k;
      x_pow_k *= x;
    }

    B(i) = y;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);
  Eigen::VectorXd x = svd.solve(B);

  return x;
}

inline std::vector<double> invPolyFit(const int &width, const int &height,
                                      const std::vector<double> &param) {

  std::vector<double> rou_vec;
  std::vector<double> z_vec;
  for (double rou = 0.0; rou <= (width + height) / 2; rou += 0.1) {
    double rou_pow_k = 1.0;
    double z = 0.0;

    for (int k = 0; k < SCARAMUZZA_POLY_SIZE; k++) {
      z += rou_pow_k * param[5 + k];
      rou_pow_k *= rou;
    }

    rou_vec.push_back(rou);
    z_vec.push_back(-z);
  }

  assert(rou_vec.size() == z_vec.size());
  Eigen::VectorXd xVec(rou_vec.size());
  Eigen::VectorXd yVec(rou_vec.size());

  for (size_t i = 0; i < rou_vec.size(); ++i) {
    xVec(i) = std::atan2(-z_vec.at(i), rou_vec.at(i));
    yVec(i) = rou_vec.at(i);
  }

  // use lower order poly to eliminate over-fitting cause by noisy/inaccurate
  // data
  Eigen::VectorXd inv_poly_coeff =
      polyfit(xVec, yVec, SCARAMUZZA_INV_POLY_SIZE - 1);

  std::vector<double> inv_poly_coeff_vec(SCARAMUZZA_INV_POLY_SIZE, 0.0);
  for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; ++i) {
    inv_poly_coeff_vec[i] = inv_poly_coeff(i);
  }

  return inv_poly_coeff_vec;
}

inline std::vector<double> generateAllParam(const int &width, const int &height,
                                            const std::vector<double> &param) {
  std::vector<double> inv_poly_coeff_vec = invPolyFit(width, height, param);

  std::vector<double> all_param(SCARAMUZZA_PARAMS_SIZE, 0.0);
  for (int i = 0; i < 5 + SCARAMUZZA_POLY_SIZE; ++i) {
    all_param[i] = param[i];
  }

  for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; ++i) {
    all_param[5 + SCARAMUZZA_POLY_SIZE + i] = inv_poly_coeff_vec[i];
  }

  return all_param;
}

} // namespace scaramuzza_functions

template <typename T>
void GeneralCameraModel::ScaramuzzaCameraModel::spaceToPlane(
    const T *params, const Eigen::Matrix<T, 3, 1> &P, Eigen::Matrix<T, 2, 1> *p,
    Eigen::Matrix<T, 2, 3> *J) {

  if (!p) {
    printf("p is null\n");
    return;
  }

  T c = params[0];
  T d = params[1];
  T e = params[2];
  T cx = params[3];
  T cy = params[4];

  T norm = ceres::sqrt(P(0) * P(0) + P(1) * P(1));
  if (ceres::abs(norm) < T(1e-14)) {
    (*p)(0) = cx;
    (*p)(1) = cy;
    return;
  }

  T theta = ceres::atan2(-P(2), norm);
  T rho = T(0.0);
  T theta_i = T(1.0);
  for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; i++) {
    rho += theta_i * params[5 + SCARAMUZZA_POLY_SIZE + i];
    theta_i *= theta;
  }

  T invNorm = T(1.0) / norm;
  T xn0 = P(0) * invNorm * rho;
  T xn1 = P(1) * invNorm * rho;

  (*p)(0) = xn0 * c + xn1 * d + cx;
  (*p)(1) = xn0 * e + xn1 + cy;

  // TODO: compute Jacobian
  if (J) {
    J->setZero();
  }
}

template <typename T>
void GeneralCameraModel::ScaramuzzaCameraModel::planeToSpace(
    const T *params, const Eigen::Matrix<T, 2, 1> &p,
    Eigen::Matrix<T, 3, 1> *P) {

  if (!P) {
    printf("P is null\n");
    return;
  }

  T c = params[0];
  T d = params[1];
  T e = params[2];
  T cx = params[3];
  T cy = params[4];
  T inv_scale = T(1.0) / (c - d * e);

  T xc1 = (p(0) - cx);
  T xc2 = (p(1) - cy);

  T xc_a0 = inv_scale * (xc1 - d * xc2);
  T xc_a1 = inv_scale * (-e * xc1 + c * xc2);

  T phi = ceres::sqrt(xc_a0 * xc_a0 + xc_a1 * xc_a1);
  T phi_i = T(1.0);
  T z = T(0.0);

  for (int i = 0; i < SCARAMUZZA_POLY_SIZE; i++) {
    z += phi_i * params[5 + i];
    phi_i *= phi;
  }

  (*P)(0) = xc_a0;
  (*P)(1) = xc_a1;
  (*P)(2) = -z;

  P->normalize();
}

std::vector<double> GeneralCameraModel::ScaramuzzaCameraModel::initializeParams(
    const double focal_length, const size_t width, const size_t height) {
  std::vector<double> params(SCARAMUZZA_PARAMS_SIZE, 0);
  params[0] = 1;                       // c
  params[3] = width / 2.0;             // cx
  params[4] = height / 2.0;            // cy
  params[5] = -(width + height) / 2.0; // p1
  return params;
}

} // namespace general_camera_model