cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/box_robot_body.cpp
  src/laser_ros.cpp
  src/pointcloud_ros.cpp
  src/light_curtain.cpp
  src/forward_velocity_filter.cpp)

rosbuild_add_library(${PROJECT_NAME}_nodelet
  src/velocity_curtain_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME})

rosbuild_add_executable(velocity_curtain src/velocity_curtain.cpp)
target_link_libraries(velocity_curtain ${PROJECT_NAME}_nodelet)

rosbuild_add_gtest(test_velocity_filter test/test_velocity_filter.cpp)
target_link_libraries(test_velocity_filter ${PROJECT_NAME})

rosbuild_add_gtest(test_box_robot_body test/test_box_robot_body.cpp)
target_link_libraries(test_box_robot_body ${PROJECT_NAME})

rosbuild_add_gtest(test_light_curtain test/test_light_curtain.cpp)
target_link_libraries(test_light_curtain ${PROJECT_NAME})
