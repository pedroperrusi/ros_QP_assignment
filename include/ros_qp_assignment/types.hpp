/**
 * @file types.h
 * @author Pedro Perrusi (pedro.perrusi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-10-20
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include <Eigen/Dense>

namespace ros_qp {

/** Eigen aliases */
using MatrixType =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MapType = Eigen::Map<MatrixType>;
using ConstMapType = Eigen::Map<const MatrixType>;

}  // namespace ros_qp