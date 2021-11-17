cmake_minimum_required(VERSION 3.0.2)
project(ur5_calibration)

find_package(catkin REQUIRED COMPONENTS backward_ros)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
