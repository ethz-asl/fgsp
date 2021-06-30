cmake_minimum_required(VERSION 3.1)
project(cdpgo)

add_definitions(-Wall)


find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY bin config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
