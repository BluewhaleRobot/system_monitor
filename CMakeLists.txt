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
  dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)


generate_dynamic_reconfigure_options(
  cfg/system_monitor.cfg
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

add_dependencies(galileo_status_relay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

message("Init system_monitor config ...")
execute_process(
  COMMAND "${PROJECT_SOURCE_DIR}/script/init.sh"
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error
  RESULT_VARIABLE result
)

if("${result}" STREQUAL "0")
  message("${output}")
  message("Init system monitor config success")
else()
  message("${error}")
  message(FATAL_ERROR "Init system monitor config failed")
endif()