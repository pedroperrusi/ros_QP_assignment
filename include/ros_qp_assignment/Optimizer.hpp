/*
 File name: Optimizer.hpp
 Author: TODO
 E-mail:
 Date created:
 Date last modified:
 */

/*
  TODO: this class should contain a ros node handle to communicate with the ROS.
  It should subscribed to "optimization_parameters" topic which has a msg type
  "Float64MultiArray". When a msg received, this class should solve the
  optimization problem with received parameters and print the result.
*/
#pragma once

#include <ceres/ceres.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

#include "ros_qp_assignment/unconstrained_qp.hpp"

/** Optimization definitions */
using ceres::FirstOrderFunction;
using ceres::GradientProblemSolver;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class Optimizer {
   public:
    Optimizer() {
        sub_ = nh_.subscribe("optimization_parameters", 10,
                             &Optimizer::paramCallback, this);
    }

    ~Optimizer() {}

    inline void run() {
        while (ros::ok()) ros::spin();
    }

    inline void paramCallback(
        const std_msgs::Float64MultiArray::ConstPtr& msg) {
        /* Deserialize coeffitients */
        ros_qp::QuadraticCoeffs coeffs(msg->data.size(), msg->data.data());
	if (!validInput(coeffs)) return;
        parameters_.resize(coeffs.order);
        std::fill(parameters_.begin(), parameters_.end(), 1);
        /** Initialize optimization routine */
        ceres::GradientProblem problem(new ros_qp::UnconstrainedQP(coeffs));
        ceres::Solve(options_, problem, parameters_.data(), &summary_);
        printResults(coeffs.order);
    }

    inline void printResults(size_t order) {
        if (summary_.termination_type != ceres::CONVERGENCE) {
            std::cout << "Solution did not converge" << std::endl;
        }
        auto solution = ros_qp::ConstMapType(parameters_.data(), order, 1);
        std::cout << "Solution:" << std::endl;
        std::cout << solution << std::endl;
        std::cout << "Convergence time:" << std::endl;
        std::cout << summary_.total_time_in_seconds << std::endl;
    }

    inline bool validInput(const ros_qp::QuadraticCoeffs& coeffs) {
      if (coeffs.isValid()) return true;
      std::cerr << "Error: Quadratic coefficients received are not valid" << std::endl;
      // std::cerr << "N: " << coeffs.order << std::endl;
      // std::cerr << "Q: \n" << coeffs.q << std::endl;
      // std::cerr << "c: \n" << coeffs.c << std::endl;
      return false;
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ceres::GradientProblemSolver::Options options_;
    ceres::GradientProblemSolver::Summary summary_;
    std::vector<double> parameters_;
};
