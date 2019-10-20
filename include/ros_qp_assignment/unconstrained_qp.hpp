/**
 * @file unconstr_quadratic.h
 * @author Pedro Perrusi (pedro.perrusi@gmail.com)
 * @brief Uncontrained quadratic function structure containing  structures
 * @version 0.1
 * @date 2019-10-20
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include "ros_qp_assignment/quadratic_coeffs.hpp"
#include "ros_qp_assignment/types.hpp"

namespace ros_qp {

/**
 * @brief Cost function for the quadratic optimization.
 *
 * Receives Eigen Map types and returns an eigen matrix type.
 *
 * @param coeffs: structure containing order, Q and C.
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
 * @param coeffs: structure containing order, Q and C.
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
 * @param coeffs: structure containing order, Q and C.
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
 * @param coeffs: structure containing order, Q and C.
 * @param parameters: input array of doubles of size coeffs.order
 * @param grad: output array of doubles of size coeffs.order
 */
inline void quadraticGrad(const QuadraticCoeffs coeffs,
                          const double *parameters, double *grad) {
    auto x = ConstMapType(parameters, coeffs.order, 1);
    auto grad_map = MapType(grad, coeffs.order, 1);
    grad_map = quadraticGrad(coeffs, x);
}

/**
 * @brief Ceres solver compatible definition of an unconstrained Quadratic
 * Optimization
 *
 */
class UnconstrainedQP : public ceres::FirstOrderFunction {
   public:
    explicit UnconstrainedQP(const QuadraticCoeffs &coeffs) : coeffs_(coeffs) {}

    virtual ~UnconstrainedQP() {}

    virtual bool Evaluate(const double *parameters, double *cost,
                          double *gradient) const {
        quadraticCost(coeffs_, parameters, cost);
        if (gradient != NULL) {
            quadraticGrad(coeffs_, parameters, gradient);
        }
        return true;
    }
    virtual int NumParameters() const { return coeffs_.order; }
    const QuadraticCoeffs coeffs_;
};

}  // namespace ros_qp
