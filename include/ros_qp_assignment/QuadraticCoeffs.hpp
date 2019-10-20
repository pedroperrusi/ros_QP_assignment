/**
 * @file quadratic_coeffs.hpp
 * @author Pedro Perrusi (pedro.perrusi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-10-20
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include <vector>
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
inline size_t inferOrder(const size_t num_elems) {
    return (-1 + sqrt(1 + 4 * num_elems)) / 2;
}

/**
 * @brief Quadratic function parameters
 *
 * Receives a serialized input of size order*(order + 1).
 * First the order of the system is infered by the number of elements.
 * Then the data is deserialized into the Hessian matrix Q and the vector C.
 *
 */
struct QuadraticCoeffs {
    size_t order;                  // quadratic program order
    std::vector<double> data_arr;  // pointer to the input coefficients
    MatrixType q;                  // Hessian matrix of shape order by order.
    MatrixType c;                  // Matrix of shape order by 1.
    /**
     * @brief Quadratic Coeffs constructor
     *
     * Infers the order of the quadratic function and creates a copy of its
     * input parameters.
     *
     * Creating a copy was preffered in case the input data is
     * not persistent.
     *
     * @param num_elems: number of elements in in_array
     * @param in_array: data array containing the serialized parameters
     */
    QuadraticCoeffs(const size_t num_elems, const double *in_array) {
        order = inferOrder(num_elems);
        data_arr.resize(num_elems);
        data_arr.assign(in_array, in_array + num_elems);
        q = ConstMapType(data_arr.data(), order, order);
        c = ConstMapType(data_arr.data() + order * order, order, 1);
    }

    /**
     * @brief Deserialize the Hessian matrix.
     *
     * @return Matrix of shape order by order.
     */
    inline MatrixType getQ() const { return q; }

    /**
     * @brief Deserialize the C vector.
     *
     * @return Matrix of shape order by 1.
     */
    inline MatrixType getC() const { return c; }

    inline bool isValid() const {
        /** Check for a positive matrix order */
        if (order <= 0) return false;  // should never happen
        /** Check if order corresponds to number of elements */
        if (order * (order + 1) != data_arr.size()) return false;
        /** Check if Q is a symmetric matrix */
        if (!q.isApprox(q.transpose())) return false;
        return true;
    }
};
}  // namespace ros_qp
