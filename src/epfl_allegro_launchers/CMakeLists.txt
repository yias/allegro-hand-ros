cmake_minimum_required(VERSION 2.8.3)
project(epfl_allegro_launchers)

find_package(catkin REQUIRED COMPONENTS
)
catkin_package()

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.launch"
)
