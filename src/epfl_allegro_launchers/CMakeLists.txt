cmake_minimum_required(VERSION 2.8.3)
project(epfl_allegro_launchers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package()


## Install launch files
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.launch"
)

