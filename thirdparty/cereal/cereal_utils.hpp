#pragma once

#include "cereal/archives/binary.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/set.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/unordered_map.hpp"
#include "cereal/types/utility.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/types/vector.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace cereal {

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline typename std::enable_if<
    traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value,
    void>::type
save(Archive &ar,
     const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
         &matrix) {
  const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
  const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
  ar(rows);
  ar(cols);
  ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
};

/**
 * @brief De-serialise an Eigen::Matrix using cereal.
 *
 * Reads the block of binary data back from a cereal archive into the
 * Eigen::Matrix.
 *
 * @param[in] ar The archive to deserialise from.
 * @param[in] matrix The matrix to deserialise into.
 */
template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline typename std::enable_if<
    traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value,
    void>::type
load(Archive &ar,
     Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
         &matrix) {
  std::int32_t rows;
  std::int32_t cols;
  ar(rows);
  ar(cols);

  matrix.resize(rows, cols);

  ar(binary_data(matrix.data(),
                 static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
};

template <class Archive> void save(Archive &ar, const cv::Mat &mat) {
  int rows = mat.rows;
  int cols = mat.cols;
  int type = mat.type();
  bool continuous = mat.isContinuous();
  bool is_empty = mat.empty();

  ar &is_empty &rows &cols &type &continuous;

  if (!is_empty) {
    if (continuous) {
      const int data_size = rows * cols * static_cast<int>(mat.elemSize());
      auto mat_data = cereal::binary_data(mat.ptr(), data_size);
      ar &mat_data;
    } else {
      const int row_size = cols * static_cast<int>(mat.elemSize());
      for (int i = 0; i < rows; i++) {
        auto row_data = cereal::binary_data(mat.ptr(i), row_size);
        ar &row_data;
      }
    }
  }
}

template <class Archive> void load(Archive &ar, cv::Mat &mat) {
  int rows, cols, type;
  bool continuous;
  bool is_empty;

  ar &is_empty &rows &cols &type &continuous;

  if (is_empty) {
    mat.release();
  } else {
    mat.create(rows, cols, type);

    if (continuous) {
      const int data_size = rows * cols * static_cast<int>(mat.elemSize());
      auto mat_data = cereal::binary_data(mat.ptr(), data_size);
      ar &mat_data;
    } else {
      const int row_size = cols * static_cast<int>(mat.elemSize());
      for (int i = 0; i < rows; i++) {
        auto row_data = cereal::binary_data(mat.ptr(i), row_size);
        ar &row_data;
      }
    }
  }
}

template <class Archive, class _Tp>
void save(Archive &ar, const cv::Point_<_Tp> &pt) {
  ar(pt.x, pt.y);
}

template <class Archive, class _Tp>
void load(Archive &ar, cv::Point_<_Tp> &pt) {
  ar(pt.x, pt.y);
}

} // namespace cereal