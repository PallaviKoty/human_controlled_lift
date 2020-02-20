cmake_minimum_required(VERSION 2.8.3)
project(elevator_plugin)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
include(FindProtobuf)
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    nodelet 
    tf 
    gazebo_plugins
    gazebo_ros
    message_generation
    geometry_msgs
    controller_interface
    control_msgs
    dynamic_reconfigure
    nav_msgs
    realtime_tools
    tf
    urdf
    pluginlib
)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

generate_dynamic_reconfigure_options(cfg/DiffDriveController.cfg)


#add services:
add_service_files(DIRECTORY srv FILES AddGroup.srv DeleteGroup.srv OpenCloseDoors.srv ListGroups.srv SetVelDoors.srv)
add_message_files(DIRECTORY msg FILES ControlGroup.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

#add catkin sourced packages:
catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet std_msgs geometry_msgs tf gazebo_plugins gazebo_ros message_runtime
)

#find and add gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${Boost_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}  
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a cpp library

add_library(SimEventsPlugin src/SimEventsPlugin.cc)
target_link_libraries(SimEventsPlugin ${catkin_LIBRARIES})


### executables, utilities for debugging atlas simulation, should not depend on gazebo

add_library(ElevatorPlugin src/ElevatorPlugin.cc)
target_link_libraries(ElevatorPlugin ${catkin_LIBRARIES})


#Controller Executables:
add_executable(dynamics_manager src/controllers/dynamics_manager.cpp src/controllers/control_group.h)
add_dependencies(dynamics_manager ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(dynamics_manager ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PROTOBUF_LIBRARY})

add_executable(keyboard_op src/controllers/keyboard_op.cpp)
add_dependencies(keyboard_op ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(keyboard_op ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

#Plugin Libraries:
add_library(door_plugin src/plugins/door_plugin.cc)
target_link_libraries(door_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PROTOBUF_LIBRARY})

add_library(${PROJECT_NAME} src/diff_drive_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#   )

install(FILES diff_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

### executables, utilities for debugging atlas simulation, should not depend on gazebo


#############
## Install ##
#############
#############
## Install ##
#############
install(TARGETS SimEventsPlugin 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ElevatorPlugin 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS dynamics_manager keyboard_op door_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
