cmake_minimum_required(VERSION 2.8.3)
project(imu_receiver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## Declare a catkin package
catkin_package()

## Build the node
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_data_logger src/imu_data_logger.cpp)
target_link_libraries(imu_data_logger ${catkin_LIBRARIES})

## Install the node
install(TARGETS imu_data_logger
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
