#include "general_camera_model/function.hpp"
#include "general_camera_model/cost_function.hpp"

namespace general_camera_model {

GeneralCameraModel cameraModelConvert(const GeneralCameraModel &old_camera,
                                      int model_id) {
  if (old_camera.modelId() == model_id) {
    std::cout << "The camera model is already " << model_id << std::endl;
    return old_camera;
  }

  std::vector<Eigen::Vector3d> pts3d;
  std::vector<Eigen::Vector2d> pts2d;

  // Get sample points
  {
    int sample_num = 31;
    int w_d = old_camera.width() / (sample_num - 1);
    int h_d = old_camera.height() / (sample_num - 1);

    for (int i = 0; i < old_camera.width(); i += w_d) {
      for (int j = 0; j < old_camera.height(); j += h_d) {
        Eigen::Vector2d point(i, j);
        pts2d.push_back(point);
        Eigen::Vector3d ray;
        old_camera.planeToSpace(point, &ray);
        pts3d.push_back(ray);
      }
    }

    if (old_camera.width() % (sample_num - 1) != 0) {
      for (int i = 0; i < old_camera.width(); i += w_d) {
        Eigen::Vector2d point(i, old_camera.height() - 1);
        pts2d.push_back(point);
        Eigen::Vector3d ray;
        old_camera.planeToSpace(point, &ray);
        pts3d.push_back(ray);
      }
    }

    if (old_camera.height() % (sample_num - 1) != 0) {
      for (int j = 0; j < old_camera.height(); j += h_d) {
        Eigen::Vector2d point(old_camera.width() - 1, j);
        pts2d.push_back(point);
        Eigen::Vector3d ray;
        old_camera.planeToSpace(point, &ray);
        pts3d.push_back(ray);
      }
    }

    Eigen::Vector2d point(old_camera.width() - 1, old_camera.height() - 1);
    pts2d.push_back(point);
    Eigen::Vector3d ray;
    old_camera.planeToSpace(point, &ray);
    pts3d.push_back(ray);
  }

  for (int i = 0; i < pts2d.size(); i++) {
    std::cout << "pts2d: " << pts2d[i].transpose()
              << " pts3d: " << pts3d[i].transpose() << std::endl;
  }

  GeneralCameraModel::Ptr new_camera(new GeneralCameraModel());
  new_camera->cameraModelInitializeParams(
      model_id, std::max(old_camera.width(), old_camera.height()) / 2.0,
      old_camera.width(), old_camera.height());
  new_camera->setCameraName(old_camera.cameraName());

  std::cout << new_camera->paramsInfo() << std::endl;
  ceres::Problem problem;
  problem.AddParameterBlock(new_camera->paramsData(), new_camera->numParams());

  for (int i = 0; i < pts2d.size(); i++) {
    auto *cost_function =
        CameraModelConvertCostFunction::Create(pts2d[i], pts3d[i], new_camera);
    std::vector<double *> params = {new_camera->paramsData()};
    problem.AddResidualBlock(cost_function, NULL, params);
  }

  for (int i = 0; i < pts2d.size(); i++) {
    auto *cost_function =
        CameraModelConvertCostFunction::Create(pts2d[i], pts3d[i], new_camera);
    auto f = new CameraModelConvertCostFunction(pts2d[i], pts3d[i], new_camera);
    std::vector<double *> params = {new_camera->paramsData()};
    double residuals[2];
    f->test(&(params[0]), residuals);

    // problem.AddResidualBlock(cost_function, NULL, params);
  }

  ceres::Solver::Options solver_options;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.minimizer_type = ceres::TRUST_REGION;
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.num_threads = 8;
  solver_options.max_num_iterations = 1000;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  new_camera->generateExtraParams();

  return *new_camera;
}

std::pair<cv::Mat, cv::Mat>
initUndistortRectifyMap(const GeneralCameraModel &old_camera,
                        const GeneralCameraModel &new_camera,
                        const Eigen::Matrix3d &R) {
  cv::Mat remap_x, remap_y;
  old_camera.initUndistortRectifyMap(new_camera, R, &remap_x, &remap_y);

  return std::make_pair(remap_x, remap_y);
}

std::vector<std::pair<int, int>>
calcOverlapViews(const std::vector<GeneralCameraModel> &camera_vec,
                 const std::vector<Eigen::Matrix4d> &ex_vec,
                 double average_depth, const double &ratio,
                 const std::string &debug_dir) {
  assert(camera_vec.size() == ex_vec.size());

  const size_t camera_num = camera_vec.size();

  int pano_camera_w = 1000;
  int pano_camera_h = 500;
  general_camera_model::GeneralCameraModel pano_camera;
  pano_camera.cameraModelInitializeParams(1, average_depth, pano_camera_w,
                                          pano_camera_h);

  Eigen::Matrix4d T0 = ex_vec[0];
  Eigen::Vector3d t0 = Eigen::Vector3d::Zero();
  for (size_t cam_id = 0; cam_id < camera_num; cam_id++) {
    Eigen::Matrix4d Ti = ex_vec[cam_id];
    t0 += Ti.block<3, 1>(0, 3);
  }
  t0 /= double(camera_num);
  T0.block<3, 1>(0, 3) = t0;

  std::vector<Eigen::Matrix4d> Tinv_vec;
  for (size_t cam_id = 0; cam_id < camera_num; cam_id++) {
    Eigen::Matrix4d Ti = ex_vec[cam_id];
    Tinv_vec.push_back(Ti.inverse() * T0);
  }

  std::vector<int> cam_sum_pts(camera_num, 0);
  std::vector<std::vector<int>> cam_common_pts(camera_num,
                                               std::vector<int>(camera_num, 0));
  std::vector<cv::Mat> cam_laps(camera_num);
  for (int i = 0; i < camera_num; i++)
    cam_laps[i] = cv::Mat(pano_camera_h, pano_camera_w, CV_8UC1, cv::Scalar(0));

  for (int v = 0; v < pano_camera_h; ++v) {
    for (int u = 0; u < pano_camera_w; ++u) {
      Eigen::Vector2d p0(u, v);
      Eigen::Vector3d P0;

      pano_camera.planeToSpace(p0, &P0);

      if (P0.norm() < 1e-6)
        continue;

      P0 = P0.normalized() * average_depth;

      std::vector<int> cam_ids;
      for (int cam_id = 0; cam_id < camera_num; cam_id++) {
        const Eigen::Matrix4d &Tinv = Tinv_vec[cam_id];
        Eigen::Vector3d P1 =
            Tinv.block<3, 3>(0, 0) * P0 + Tinv.block<3, 1>(0, 3);

        if (P1(2) <= 0)
          continue;

        Eigen::Vector2d p1(-1, -1);
        camera_vec[cam_id].spaceToPlane(P1, &p1);

        if (!camera_vec[cam_id].isPixelVaild(p1))
          continue;

        cam_ids.push_back(cam_id);
      }

      for (int i = 0; i < cam_ids.size(); i++) {
        cam_sum_pts[cam_ids[i]]++;
        cam_laps[cam_ids[i]].at<uchar>(v, u) = 255;
        for (int j = i + 1; j < cam_ids.size(); j++) {
          cam_common_pts[cam_ids[i]][cam_ids[j]]++;
          cam_common_pts[cam_ids[j]][cam_ids[i]]++;
        }
      }
    }
  }

  if (debug_dir != "") {
    for (int cam_id = 0; cam_id < camera_num; cam_id++) {
      cv::imwrite(debug_dir + "/cam_lap_" + std::to_string(cam_id) + ".png",
                  cam_laps[cam_id]);
    }
  }

  std::vector<std::pair<int, int>> stereo_map;
  for (int cam_id_i = 0; cam_id_i < camera_num; cam_id_i++) {
    for (int cam_id_j = cam_id_i + 1; cam_id_j < camera_num; cam_id_j++) {
      if (cam_id_i == cam_id_j)
        continue;

      double overlap_ratio1 = double(cam_common_pts[cam_id_i][cam_id_j]) /
                              double(cam_sum_pts[cam_id_i]);
      double overlap_ratio2 = double(cam_common_pts[cam_id_i][cam_id_j]) /
                              double(cam_sum_pts[cam_id_j]);
      // std::cout << std::fixed << cam_id_i << "," << cam_id_j
      //           << " overlap_ratio1: " << overlap_ratio1
      //           << " overlap_ratio2: " << overlap_ratio2 << std::endl;

      if (overlap_ratio1 > ratio && overlap_ratio2 > ratio) {
        stereo_map.push_back(std::make_pair(cam_id_i, cam_id_j));
      }
    }
  }

  return stereo_map;
}

GeneralCameraModel getSimplePinhole(double fx, double fy, double cx, double cy,
                                    int width, int height) {
  GeneralCameraModel camera;
  camera.cameraModelInitializeParams(2, 1000, width, height);
  camera.setCameraName("simple_pinhole");
  camera.setParams({fx, fy, cx, cy, 0, 0, 0, 0});
  return camera;
}

std::vector<Eigen::Vector2d>
batchSpaceToPlane(const GeneralCameraModel &camera,
                  const std::vector<Eigen::Vector3d> &pts) {
  std::vector<Eigen::Vector2d> plane_pts;
  for (const auto &pt : pts) {
    Eigen::Vector2d plane_pt;
    camera.spaceToPlane(pt, &plane_pt);
    plane_pts.push_back(plane_pt);
  }
  return plane_pts;
}

std::vector<Eigen::Vector3d>
batchPlaneToSpace(const GeneralCameraModel &camera,
                  const std::vector<Eigen::Vector2d> &pts) {
  std::vector<Eigen::Vector3d> space_pts;
  for (const auto &pt : pts) {
    Eigen::Vector3d space_pt;
    camera.planeToSpace(pt, &space_pt);
    space_pts.push_back(space_pt);
  }
  return space_pts;
}

} // namespace general_camera_model
