cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner_rqt)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_py
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rqt_gui rqt_gui_py rospy std_msgs geometry_msgs visualization_msgs
)
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/trajectory_planner_rqt/trajectory_planner_rqt.py
  #DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src/trajectory_planner_rqt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  resource/trajectory_planner_rqt.ui
  #config/trajectory_planner.yaml
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

