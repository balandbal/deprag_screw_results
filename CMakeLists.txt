cmake_minimum_required(VERSION 3.0.2)
project(deprag_downloader)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cppitasc
)

 add_message_files(
   FILES
   screwing.msg
 )

 add_service_files(
   FILES
    download_request.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  LIBRARIES deprag_downloader
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime  cppitasc
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/script_download_curve.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
