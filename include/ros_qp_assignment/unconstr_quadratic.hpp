/**
 * @file unconstr_quadratic.h
 * @author Pedro Perrusi (pedro.perrusi@gmail.com)
 * @brief Uncontrained quadratic function definitions and structures
 * @version 0.1
 * @date 2019-10-20
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include "ros_qp_assignment/types.hpp"

namespace ros_qp {

/**
 * @brief Based on the number of elements in an input array, infer what is the
 * order of the quadratic optimization.
 *
 * The number of elements is a function of the system order:
 *      num_elems = order*(order + 1)
 *  As num_elems is always positive, the equation assumes only one positive
 * integer solution:
 *      order = (-1 + sqrt(1 + 4*num_elems))/2.
 *
 * @param num_elems: number of elements in a quadratic function parameter array
 * @return order of the quadratic function.
 */
inline size_t inferOrder(size_t num_elems) {
    return (-1 + sqrt(1 + 4 * num_elems)) / 2;
}

/**
 * @brief Quadratic function parameters
 *
 */
struct QuadraticCoeffs {
    size_t order;  // quadratic program order
    double *data_ptr;  // pointer to the input coefficients
    QuadraticCoeffs(size_t num_elems, double *input) : data_ptr(input) {
        order = inferOrder(num_elems);
    }
    MatrixType getQ() const { return MapType(data_ptr, order, order); }
    MatrixType getC() const {
        return MapType(data_ptr + order * order, order, 1);
    }
};

/**
 * @brief Cost function for the quadratic optimization.
 *
 * Receives Eigen Map types and returns an eigen matrix type.
 *
 * @param coeffs: QuadraticCoeffs definitions of order, Q and C.
 * @param x: Eigen type collumn vector of order coeffs.order.
 * @return MatrixType: Scalar interpreted as Matrix 1x1
 */
inline MatrixType quadraticCost(const QuadraticCoeffs coeffs, ConstMapType x) {
    return 0.5 * x.transpose() * coeffs.getQ() * x +
           coeffs.getC().transpose() * x;
}

/**
 * @brief Cost function for the quadratic optimization.
 *
 * Operator overload in accordance with Ceres input and outputs
 *
 * @param coeffs: QuadraticCoeffs definitions of order, Q and C.
 * @param parameters: input array of doubles of size coeffs.order
 * @param cost: output array of doubles of size 1
 */
inline void quadraticCost(const QuadraticCoeffs coeffs,
                          const double *parameters, double *cost) {
    auto x = ConstMapType(parameters, coeffs.order, 1);
    auto cost_map = MapType(cost, 1, 1);
    cost_map = quadraticCost(coeffs, x);
}

/**
 * @brief Gradient function for the quadratic optimization.
 *
 * Inputs shoud be of dimension coeffs.order.
 * Output is a scalar.
 *
 * @param coeffs: QuadraticCoeffs definitions of order, Q and C.
 * @param x: Eigen type collumn vector of order coeffs.order.
 * @return MatrixType: Eigen type collumn vector of order coeffs.order.
 */
inline MatrixType quadraticGrad(const QuadraticCoeffs coeffs, ConstMapType x) {
    return coeffs.getQ() * x + coeffs.getC();
}

/**
 * @brief Gradient function for the quadratic optimization.
 *
 * Operator overload in accordance with Ceres input and outputs
 *
 * @param coeffs: QuadraticCoeffs definitions of order, Q and C.
 * @param parameters: input array of doubles of size coeffs.order
 * @param grad: output array of doubles of size coeffs.order
 */
inline void quadraticGrad(const QuadraticCoeffs coeffs,
                          const double *parameters, double *grad) {
    auto x = ConstMapType(parameters, coeffs.order, 1);
    auto grad_map = MapType(grad, coeffs.order, 1);
    grad_map = quadraticGrad(coeffs, x);
}

}  // namespace ros_qp
