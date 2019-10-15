# ros_QP_assignment


## Assignment
In this assignment, you are asked to write a cpp header, which calculate following quadratic optimization with the parameters coming from ROS message. The quadratic optimization is in the following form.

```math
min 1/2 x^TQx +c^Tx
```
where Q is a n by n matrix and c is a 1 by n vector. These parameters will be send to your program through a publisher with name "/optimization_parameters" with msg type [Float64MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64MultiArray.html). The msg will contain n*(n+1) parameter for each Q and c. The first n*n parameters are the elements of Q matrix and the Q matrix is flattened row by row (flatten(Q) = (row1, row2,..., rown) )








## Dependencies
* [Ros Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (or Kinetic)
* [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
* Eigen3
* [Ceres](http://ceres-solver.org/installation.html)


## How to run test code
If you dont have a "catkin_ws", create one

```bash
mkdir ~/catkin_ws
mkdir ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

Then, clone the repository to your "catkin_ws/src"
```bash
cd ~/catkin_ws/src
git clone https://github.com/MEfeTiryaki/ros_QP_assignment.git
```

build the code
```bash
cd ~/catkin_ws
catkin build ros_QP_assignment
```

source it

```bash
cd ~/catkin_ws
source devel/setup.bash
```

run the test
```bash
rosrun ros_QP_assignment ceres_test
```
