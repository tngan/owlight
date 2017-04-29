cmake_minimum_required(VERSION 2.8.3)
project(owlight_node)
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
)
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
#add_executable(${PROJECT_NAME} scripts/owlight_node.py)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
