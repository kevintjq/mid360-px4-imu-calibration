cmake_minimum_required(VERSION 3.0.2)
project(mid360_px4_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  topic_tools
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib sensor_msgs topic_tools pcl_ros pcl_conversions
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(online_calibration_node src/online_calibration_node.cpp)
target_compile_features(online_calibration_node PRIVATE cxx_std_14)
target_link_libraries(online_calibration_node ${catkin_LIBRARIES})

install(TARGETS online_calibration_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
