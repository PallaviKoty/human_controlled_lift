cmake_minimum_required(VERSION 2.8.3)
project(robot_position_plugin)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")


find_package(catkin REQUIRED COMPONENTS
             tf
             tf2_ros
             roscpp
             std_msgs
             nav_msgs
)



catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 gazebo_ros_p3d
 CATKIN_DEPENDS
 std_msgs
 nav_msgs
 tf
 tf2_ros
#  DEPENDS system_lib
)
# add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

find_package(gazebo REQUIRED)

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


add_library(gazebo_ros_p3d src/gazebo_ros_p3d.cc)
target_link_libraries(gazebo_ros_p3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})



## Declare a cpp library



# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
#   catkin_package(  INCLUDE_DIRS include LIBRARIES gazebo_ros_elevator)
# endif()


# if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
#   add_library(gazebo_ros_elevator src/Elevator.cc)
#   add_dependencies(gazebo_ros_elevator ${PROJECT_NAME}_gencfg)
#   target_link_libraries(gazebo_ros_elevator ElevatorPlugin ${catkin_LIBRARIES})
# endif()

# if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
#   install(TARGETS gazebo_ros_elevator
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     )
# endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
