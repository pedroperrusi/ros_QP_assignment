#!/usr/bin
# This file was designed to be called from within a docker container
# Commands inpired by: http://ceres-solver.org/installation.html#linux

# Basic installation
apt-get update
apt-get install -y build-essential
sudo apt-get install python-catkin-tools

## Installation of Ceres Solver dependencies
#
# CMake
apt-get install cmake
# google-glog + gflags
apt-get install libgoogle-glog-dev
# BLAS & LAPACK
apt-get install libatlas-base-dev
# Eigen3
apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
apt-get install libsuitesparse-dev
# - However, if you want to build Ceres as a *shared* library, you must
#   add the following PPA:
add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
apt-get update
apt-get install libsuitesparse-dev

