cmake_minimum_required(VERSION 3.0.2)
project(cube_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions visualization_msgs
    )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cube_publisher_node src/cube_publisher.cpp)
target_link_libraries(cube_publisher_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS cube_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

