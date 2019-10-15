# ros_QP_assignment


## Assignment
In this assignment, you are asked to write a c++ header, which calculates following quadratic optimization with the parameters coming from ROS message using Ceres library.

```math
min 1/2 x^TQx + c^Tx
```

where Q is a n by n matrix and c is a 1 by n vector. These parameters will be send to your program through a publisher named "/optimization_parameters" with msg type [Float64MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64MultiArray.html). The msg will contain n*(n+1) parameter for each Q and c. The first n*n parameters are the elements of Q matrix and the Q matrix is flattened row by row (flatten(Q) = (row_1, row_2,..., row_n) ). Last n element of the msg corresponds to the c vector.

The dimension of optimization will depend on the dimension of the parameters received in ros msg. We are going to test your code with n<=6. In ideal case, the optimization should be writen parametrically such that it works with any positive integer value of n. But hard coded switch/case structure is also accepted (for this case your code should send a warning message if size of the optimization is not covered by the code).

Your callback function should check whether
* the dimension of the msg data is n*(n+1) and
* the Q matrix is symmetric.
If the conditions are not satisfied, it should send an error message and skip optimization.


After the optimization, the code should print out solution and computation time.

### Final submision
You should submit a copy of this repository with modified "optimizer_main.cpp", "Optimizer.hpp", "CMakelist.txt" and others if you need. To test your code, we will simply run following

```bash
rosrun ros_QP_assignment optimizer_main
```

and test it with different parameters send from "rostopic pub"

### test input

```bash
rostopic pub "/optimization_parameters" std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1,0,0,2,2,1]" 
```
in this test Q = diag(1,2) , c = (2,1) 





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

launch roscore in a second terminal and run the test
```bash
rosrun ros_QP_assignment ceres_test
```
