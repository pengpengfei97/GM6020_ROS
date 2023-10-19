cmake_minimum_required(VERSION 3.0.2)
project(robomaster_gm6020_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

find_package(PkgConfig)
find_package(Boost REQUIRED COMPONENTS system)
pkg_check_modules(APR apr-1) 
if (APR_FOUND) 
	message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES}) 
endif (APR_FOUND)

add_message_files(
  FILES
  MotorMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES robomaster_gm6020_ros
 CATKIN_DEPENDS roscpp rospy std_msgs tf
 # DEPENDS system_lib
)



###########
## Build ##
###########
# for ROS
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Other includes
include_directories(
    ${APR_INCLUDE_DIRS}               
    /usr/local/include
)


#############
## Install ##
#############

add_executable(motor_ctr_node src/motor_ctr.cpp)
target_link_libraries(
    motor_ctr_node
    ${APR_LIBRARIES}
    ${catkin_LIBRARIES}
    )
