cmake_minimum_required(VERSION 2.8.3)
project(osr_control)

find_package(catkin REQUIRED COMPONENTS
  effort_controllers
  gripper_action_controller
  joint_state_controller
  joint_trajectory_controller
  osr_description
  robot_state_publisher
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
