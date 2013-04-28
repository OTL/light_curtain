# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(light_curtain)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp tf pcl pcl_ros nodelet std_msgs sensor_msgs geometry_msgs laser_geometry dynamic_reconfigure)

# need cfg/cpp, is this bug?
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/VelocityCurtain.cfg)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
  src/box_robot_body.cpp
  src/laser_ros.cpp
  src/pointcloud_ros.cpp
  src/light_curtain.cpp
  src/forward_velocity_filter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_nodelet
  src/velocity_curtain_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_nodelet light_curtain_gencfg)

add_executable(velocity_curtain src/velocity_curtain.cpp)
target_link_libraries(velocity_curtain ${PROJECT_NAME}_nodelet)

catkin_add_gtest(test_velocity_filter test/test_velocity_filter.cpp)
target_link_libraries(test_velocity_filter ${PROJECT_NAME})

catkin_add_gtest(test_box_robot_body test/test_box_robot_body.cpp)
target_link_libraries(test_box_robot_body ${PROJECT_NAME})

catkin_add_gtest(test_light_curtain test/test_light_curtain.cpp)
target_link_libraries(test_light_curtain ${PROJECT_NAME})

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS roscpp tf pcl pcl_ros nodelet std_msgs sensor_msgs geometry_msgs laser_geometry dynamic_reconfigure
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet velocity_curtain
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
