# Docker environment for ros_QP_assignment
FROM pedroperrusi/ros:qp_assignment
MAINTAINER Pedro Perrusi pedroperrusi@gmail.com
WORKDIR /root/catkin_ws/
# Copy this repository to its target position
ADD ./ /root/catkin_ws/src/
RUN catkin build