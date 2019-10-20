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
#include "ceres/ceres.h"
#include <Eigen/Dense>
#include <iostream>

/** Optimization definitions */
using ceres::FirstOrderFunction;
using ceres::GradientProblemSolver;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

/** Eigen aliases */
using VecType = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor>;
using MatrixType =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MapType = Eigen::Map<MatrixType>;
using ConstMapType = Eigen::Map<const MatrixType>;

/**
 * @brief Quadratic function parameters
 *
 */
struct QuadraticCoeffs {
  size_t order;
  double *data_ptr;
  QuadraticCoeffs(size_t n, double *input) : order(n), data_ptr(input) {}
  MatrixType getQ() const { return MapType(data_ptr, order, order); }
  MatrixType getC() const { return MapType(data_ptr + order * order, order, 1); }
};

class UnconstrQuadratic : public ceres::FirstOrderFunction {
public:
  explicit UnconstrQuadratic(const QuadraticCoeffs &coeffs) : coeffs_(coeffs) {}

  virtual ~UnconstrQuadratic() {}

  virtual bool Evaluate(const double *parameters, double *cost,
                        double *gradient) const {
    auto x = ConstMapType(parameters, coeffs_.order, 1);
    auto cost_map = MapType(cost, 1, 1);
    cost_map =
        0.5 * x.transpose() * coeffs_.getQ() * x + coeffs_.getC().transpose() * x;
    if (gradient != NULL) {
      auto gradient_map = MapType(gradient, coeffs_.order, 1);
      gradient_map = coeffs_.getQ() * x + coeffs_.getC();
    }
    return true;
  }
  virtual int NumParameters() const { return coeffs_.order; }
  const QuadraticCoeffs coeffs_;
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  size_t order = 3;
  double parameters[3] = {-100.4,  400.0, 600.0123};
  double data[] = {5, -2, -1, -2, 4, 3, -1, 3, 5, 2, -35, -47};
  QuadraticCoeffs coeffs(order, data);
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
  std::cout << "Final   x: " << parameters[0] << " y: " << parameters[1] << " z: " << parameters[2]
            << "\n";
  return 0;
}
