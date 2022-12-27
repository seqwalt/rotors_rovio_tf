cmake_minimum_required(VERSION 2.8.3)
project(rotors_rovio_tf)

## Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg nav_msgs tf2_geometry_msgs tf2)

## Declare a catkin package
catkin_package()

## Include directories and add executable
set(ROVIO_SRV_HEADERS ../../devel/include)
set(HEADER_FILES ${ROVIO_SRV_HEADERS}/rovio/SrvResetToPose.h)
add_executable(est_odom_node src/est_odom_node.cpp ${HEADER_FILES})
target_link_libraries(est_odom_node ${catkin_LIBRARIES})

target_include_directories(est_odom_node PRIVATE ${ROVIO_SRV_HEADERS})
include_directories(include ${catkin_INCLUDE_DIRS} ${ROVIO_SRV_HEADERS})
