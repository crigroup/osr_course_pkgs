cmake_minimum_required(VERSION 2.8.3)
project(osr_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  osr_control
  osr_description
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
