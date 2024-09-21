cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  kdl_parser
  trajectory_msgs
  visualization_msgs
  pcl_ros
  tf
  tf2_ros
  urdf
  interactive_markers
  std_srvs
  controller_manager
  joint_trajectory_controller
)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ompl REQUIRED)
find_package(toppra REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_planner
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs kdl_parser trajectory_msgs visualization_msgs pcl_ros tf tf2_ros urdf interactive_markers std_srvs
  DEPENDS yaml-cpp ompl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${toppra_INCLUDE_DIRS}
)

add_executable(trajectory_planner_node src/trajectory_planner.cpp)
target_link_libraries(trajectory_planner_node PUBLIC ${catkin_LIBRARIES} ${OMPL_LIBRARIES} yaml-cpp toppra::toppra ${toppra_LIBRARIES})

