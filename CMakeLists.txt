cmake_minimum_required(VERSION 2.8.3)
project(ros_qp_assignment)

add_definitions(-std=c++11 -Wall -Werror)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


# Test  executable
add_executable(ceres_test
  test/ceres_test.cpp
)
target_link_libraries(ceres_test
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# Test  quadratic
add_executable(quadratic_test
  test/quadratic_test.cpp
)

target_include_directories(quadratic_test
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include/
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(quadratic_test
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# TODO : Your executable
#add_executable(optimizer_main
#  src/optimizer_main.cpp
#)

#target_link_libraries(optimizer_main
#  ${catkin_LIBRARIES}
#)
