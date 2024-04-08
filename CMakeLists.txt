cmake_minimum_required(VERSION 3.0.2)
project(traj_gen)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
  mode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES 
  traj_gen
 CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs
  message_runtime
#  DEPENDS system_lib
)

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(traj_gen_node  include/traj_gen/traj_gen.cpp
                              include/traj_gen/double_integral_planner.cpp
                              include/traj_gen/yaml_read.cpp
                              src/traj_generator_node.cpp)

target_link_libraries(traj_gen_node ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})