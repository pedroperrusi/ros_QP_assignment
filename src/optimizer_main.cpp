
#include <iostream>

#include "ros_qp_assignment/Optimizer.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "optimizer_node");
    // your optimizer class
    Optimizer optimizer = Optimizer();

    // runs as long as roscore is running
    optimizer.run();

    return 0;
}
