cmake_minimum_required(VERSION 2.8.3)
project(osr_openrave)

find_package(catkin REQUIRED COMPONENTS
  openrave_catkin
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY objects robots worlds DESTINATION ${OpenRAVE_DEVEL_DIR}/${OpenRAVE_DATA_DIR})
install(DIRECTORY objects robots worlds DESTINATION ${OpenRAVE_INSTALL_DIR}/${OpenRAVE_DATA_DIR})
