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

## Declare a cpp executable
add_executable(ball_follower_node src/BallFollowerRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ball_follower_node
   ${catkin_LIBRARIES}
)
