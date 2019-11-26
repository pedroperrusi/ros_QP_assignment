/**
 * @file talker_node.cpp
 * @author Pedro Perrusi (pedro.perrusi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-10-20
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

const double ParameterList[] = {1, 0, 0, 2, 2, 1};

int main(int argc, char** argv) {
    ros::init(argc, argv, "optimizer_talker");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(
        "optimization_parameters", 10);
    std_msgs::Float64MultiArray msg;
    ros::Rate rate(1);  // 1Hz
    while (ros::ok()) {
        size_t num_elems = sizeof(ParameterList) / sizeof(ParameterList[0]);
        msg.data.resize(num_elems);
        msg.data.assign(ParameterList, ParameterList + num_elems);
        pub.publish(msg);
        rate.sleep();
    }
    std::cout << "Talker node exit" << std::endl;
}
