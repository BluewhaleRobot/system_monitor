cmake_minimum_required(VERSION 2.8.3)
project(system_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  rospy
  std_msgs
  message_generation
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

)