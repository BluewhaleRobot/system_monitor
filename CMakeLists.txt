cmake_minimum_required(VERSION 2.8.3)
project(system_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  rospy
  roscpp
  std_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

add_message_files(
   FILES
   Status.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs message_generation message_runtime
)

add_executable(galileo_status_relay
  ${PROJECT_SOURCE_DIR}/src/galileo_status_relay.cpp
)

target_link_libraries(galileo_status_relay
   ${catkin_LIBRARIES}
)