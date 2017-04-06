cmake_minimum_required(VERSION 2.8.3)
project(osr_examples)

find_package(catkin REQUIRED COMPONENTS
  osr_control
  osr_description
  osr_gazebo
  osr_openrave
)

catkin_package(CATKIN_DEPENDS 
  osr_control
  osr_description
  osr_gazebo
  osr_openrave
)
