cmake_minimum_required(VERSION 2.8.3)
project(limo_gazebo_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
  roslaunch
  roslint
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  gazebo_dev
  gazebo_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  gazebo_ros_ackerman
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(gazebo_ros_ackerman src/gazebo_ros_ackerman_drive.cpp)
target_link_libraries(gazebo_ros_ackerman ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_dynamic_sun src/gazebo_dynamic_sun.cpp)
target_link_libraries(gazebo_dynamic_sun ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_world_light src/gazebo_world_light.cpp)
target_link_libraries(gazebo_world_light ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS gazebo_ros_ackerman gazebo_dynamic_sun gazebo_world_light
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

