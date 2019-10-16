# ros_qp_assignment


## Assignment
In this assignment, you are asked to write a cpp header, which calculate

http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64MultiArray.html

```math #yourmathlabel
a + b = c
```



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
git clone https://github.com/MEfeTiryaki/ros_qp_assignment.git
```

build the code
```bash
cd ~/catkin_ws
catkin build ros_qp_assignment
```

source it

```bash
cd ~/catkin_ws
source devel/setup.bash
```

run the test
```bash
rosrun ros_qp_assignment ceres_test
```
