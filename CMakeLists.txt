cmake_minimum_required(VERSION 3.5)
project(neo_usboard_v2)

## By default build with code optimization and no debug info
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(neo_srvs2 REQUIRED)

add_subdirectory(pilot-base)
add_subdirectory(pilot-usboard)

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  DIRECTORY msg
#  FILES EmergencyStopState.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES std_msgs
#)

###################################
## catkin specific configuration ##
###################################

ament_export_include_directories(generated/include)

ament_export_dependencies(std_msgs neo_srvs2 tf nav_msgs sensor_msgs)

###########
## Build ##
###########

file(GLOB GENERATED_SOURCE_FILES generated/src/*.cpp)

include_directories(
  generated/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(neo_usboard_v2_node src/neo_usboard_v2_node.cpp ${GENERATED_SOURCE_FILES})

ament_target_dependencies(neo_usboard_v2_node 
rclcpp
sensor_msgs
std_msgs
neo_msgs2 
neo_srvs2
)

target_link_libraries(neo_usboard_v2_node pilot_usboard pilot_base vnx_base)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS neo_usboard_v2_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()