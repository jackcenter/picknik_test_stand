cmake_minimum_required(VERSION 3.0.2)
project(picknik_test_stand_kinematic_calibration)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED 
  COMPONENTS 
    backward_ros
    robot_calibration_msgs
    roscpp)

catkin_package(
  CATKIN_DEPENDS 
    robot_calibration_msgs
    roscpp
  LIBRARIES
    robot_calibration
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(calibration_data_visualization src/calibration_data_visualization.cpp)
target_link_libraries(calibration_data_visualization
  ${catkin_LIBRARIES}) 
add_dependencies(calibration_data_visualization robot_calibration_msgs_generate_messages_cpp)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
