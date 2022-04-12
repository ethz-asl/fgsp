cmake_minimum_required(VERSION 3.1)
project(fgsp)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)


install(DIRECTORY
  config src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
