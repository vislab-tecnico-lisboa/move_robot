cmake_minimum_required(VERSION 2.8.3)
project(move_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
  image_transport
  pcl_ros
  tf_conversions
  message_generation
  nodelet
  actionlib_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  move_robot_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foveated_stereo_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
#   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #${CUDA_INCLUDE_DIRS}
)

add_executable(gaze_sim src/Gaze.cpp src/GazeSimulation.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze_sim
   ${catkin_LIBRARIES}
)

add_executable(gaze_real src/Gaze.cpp src/GazeReal.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze_real
   ${catkin_LIBRARIES}
)

add_executable(gaze_client src/GazeClient.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze_client
   ${catkin_LIBRARIES}
)

## Declare a cpp library
add_library(fixation_point_lib
   src/FixationPoint.cpp
)

## Declare a cpp executable
add_executable(fixation_point_node src/FixationPointRos.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(fixation_point_node visualize_ball_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fixation_point_node
   ${catkin_LIBRARIES}
    fixation_point_lib
)




