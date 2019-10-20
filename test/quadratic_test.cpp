/**
 * @file quadratic_test.cpp
 * @author Pedro Perrusi (pedro.perrusi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-10-19
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <Eigen/Dense>
#include <iostream>
#include "ceres/ceres.h"

#include "ros_qp_assignment/types.hpp"
#include "ros_qp_assignment/unconstrained_qp.hpp"

/** Optimization definitions */
using ceres::FirstOrderFunction;
using ceres::GradientProblemSolver;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

using namespace ros_qp;

class UnconstrQuadratic : public ceres::FirstOrderFunction {
   public:
    explicit UnconstrQuadratic(const QuadraticCoeffs &coeffs)
        : coeffs_(coeffs) {}

    virtual ~UnconstrQuadratic() {}

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

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    double parameters[3] = {1, 1, 1};
    double data[] = {5, -2, -1, -2, 4, 3, -1, 3, 5, 2, -35, -47};
    size_t num_elems = sizeof(data) / sizeof(data[0]);
    QuadraticCoeffs coeffs(num_elems, data);
    std::cout << "Q = \n" << coeffs.getQ() << std::endl;
    std::cout << "c = \n" << coeffs.getC() << std::endl;
    ceres::GradientProblemSolver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.line_search_direction_type = ceres::BFGS;
    //   options.use_approximate_eigenvalue_bfgs_scaling = true;
    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UnconstrQuadratic(coeffs));
    ceres::Solve(options, problem, parameters, &summary);
    std::cout << summary.FullReport() << "\n";
    std::cout << "Initial x: " << 1.0 << " y: " << 1.0 << " z: " << 1.0 << "\n";
    std::cout << "Final   x: " << parameters[0] << " y: " << parameters[1]
              << " z: " << parameters[2] << "\n";
    return 0;
}
