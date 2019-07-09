cmake_minimum_required(VERSION 2.8.3)
project(ackermann_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CheckpointMsg.msg
  ControlMsg.msg
  GoalMsg.msg
  ObjectMsg.msg
  ObjectStateMsg.msg
  PositionMsg.msg
  StateMsg.msg
  TransmissionMsg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ackermann_nav
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ackermann_nav.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ackermann_nav_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ackermann_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# ackermann_controller node 
add_executable(ackermann_controller src/nodes/ackermann_controller.cpp)
target_link_libraries(ackermann_controller ${catkin_LIBRARIES})

# ackermann_odom node 
add_executable(ackermann_odom src/nodes/ackermann_odom.cpp)
target_link_libraries(ackermann_odom ${catkin_LIBRARIES})

# ackermann_teleop node 
add_executable(ackermann_teleop src/nodes/ackermann_teleop.cpp)
target_link_libraries(ackermann_teleop ${catkin_LIBRARIES})

# current_state node 
add_executable(current_state src/nodes/current_state.cpp)
target_link_libraries(current_state ${catkin_LIBRARIES})

# global_map node 
add_executable(global_map src/nodes/global_map.cpp)
target_link_libraries(global_map ${catkin_LIBRARIES})

# global_plan node 
add_executable(global_plan src/nodes/global_plan.cpp)
target_link_libraries(global_plan ${catkin_LIBRARIES})

# local_plan node 
add_executable(local_plan src/nodes/local_plan.cpp)
target_link_libraries(local_plan ${catkin_LIBRARIES})

# obstacle_locations node 
add_executable(obstacle_locations src/nodes/obstacle_locations.cpp)
target_link_libraries(obstacle_locations ${catkin_LIBRARIES})



# pub1 node, publishes to single topic 
add_executable(pub1 src/templates/pub1.cpp)
target_link_libraries(pub1 ${catkin_LIBRARIES})

# pub1_2 node, publishes to single topic 
add_executable(pub1_2 src/templates/pub1_2.cpp)
target_link_libraries(pub1_2 ${catkin_LIBRARIES})

# pub1_3 node, publishes to single topic 
add_executable(pub1_3 src/templates/pub1_3.cpp)
target_link_libraries(pub1_3 ${catkin_LIBRARIES})

# sub1_pub1 node, subscribes to one topic and publishes to one topic  
add_executable(sub1_pub1 src/templates/sub1_pub1.cpp)
target_link_libraries(sub1_pub1 ${catkin_LIBRARIES})

# sub3_pub2 node, subscribes to three topics and publishes to two topics  
add_executable(sub3_pub2 src/templates/sub3_pub2.cpp)
target_link_libraries(sub3_pub2 ${catkin_LIBRARIES})

# original ackermann steering node
add_executable(ackermann_teleop_original src/templates/ackermann_teleop_original.cpp)
target_link_libraries(ackermann_teleop_original ${catkin_LIBRARIES})

# copy cat node, subscribes to talker and publishes data to new topic
add_executable(copy_cat src/templates/copy_cat.cpp)
target_link_libraries(copy_cat ${catkin_LIBRARIES})

# copy_cat_2 node, subscribes to talker, talker_2 
add_executable(copy_cat_2 src/templates/copy_cat_2.cpp)
target_link_libraries(copy_cat_2 ${catkin_LIBRARIES})

# copy_cat_2_pub node, subscribes to talker, talker_2 and publishes to new topic
add_executable(copy_cat_2_pub src/templates/copy_cat_2_pub.cpp)
target_link_libraries(copy_cat_2_pub ${catkin_LIBRARIES})

# copy_cat_3_pub2 node, subscribes to talker, talker_2, talker_3 and publishes to new topic
add_executable(copy_cat_3_pub2 src/templates/copy_cat_3_pub2.cpp)
target_link_libraries(copy_cat_3_pub2 ${catkin_LIBRARIES})

# talker node
add_executable(talker src/templates/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

# talker_2 node
add_executable(talker_2 src/templates/talker_2.cpp)
target_link_libraries(talker_2 ${catkin_LIBRARIES})

# talker_3 node
add_executable(talker_3 src/templates/talker_3.cpp)
target_link_libraries(talker_3 ${catkin_LIBRARIES})





