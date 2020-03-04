cmake_minimum_required(VERSION 2.8.3)
project(lidar_avoidance)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(laserscan_avoidance_node src/laserscan_avoidance_node.cpp src/laserscan_avoidance_activity.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(laserscan_avoidance_node ${catkin_LIBRARIES})

install(TARGETS laserscan_avoidance_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

