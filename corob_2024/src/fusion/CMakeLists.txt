cmake_minimum_required(VERSION 3.0.2)
project(fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  behaviortree_cpp
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ActivateGripper.srv
  Confirm.srv
  ConfirmTarget.srv
  ControllerStop.srv
  Feedback.srv
  GetTools.srv
  Inspection.srv
  Interrupt.srv
  IsFault.srv
  IsObjectDetected.srv
  ListTargets.srv
  ListTools.srv
  ResetProgress.srv
  SnapToTool.srv
  TaskInstructions.srv
  TaskProgress.srv
  VerifyTask.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
  roscpp
)

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
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs actionlib_msgs std_msgs behaviortree_cpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME} 
  include/fusion_tree/fusion_tree.cpp 
  include/fusion_tree/feedback_action_node.cpp 

  include/fusion_tree/services/confirm_service.cpp
  include/fusion_tree/services/confirm_target_service.cpp
  include/fusion_tree/services/feedback_service.cpp
  include/fusion_tree/services/get_task_tools_service.cpp
  include/fusion_tree/services/gripper_service.cpp
  include/fusion_tree/services/is_fault_service.cpp 
  include/fusion_tree/services/is_object_detected_service.cpp
  include/fusion_tree/services/list_targets_service.cpp
  include/fusion_tree/services/list_tools_service.cpp
  include/fusion_tree/services/reset_progress_service.cpp
  include/fusion_tree/services/snap_to_tool_service.cpp
  include/fusion_tree/services/stop_service.cpp
  include/fusion_tree/services/task_inspection_service.cpp
  include/fusion_tree/services/task_instructions_service.cpp
  include/fusion_tree/services/task_progress_service.cpp
  include/fusion_tree/services/verify_task_service.cpp

  include/fusion_tree/actions/give_action_client.cpp
  include/fusion_tree/actions/move_action_client.cpp
  include/fusion_tree/actions/pick_action_client.cpp
  include/fusion_tree/actions/place_action_client.cpp
  include/fusion_tree/actions/snap_action_client.cpp

  include/fusion_tree/conditions/is_grasping.cpp
  include/fusion_tree/conditions/is_locked.cpp
  include/fusion_tree/conditions/is_command.cpp
  include/fusion_tree/conditions/is_task_done.cpp
  include/fusion_tree/conditions/is_assembly_started.cpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
  ${PROJECT_NAME}_main_tree_node src/fusion_bt.cpp 
  include/fusion_tree/fusion_tree.cpp 
  include/fusion_tree/feedback_action_node.cpp 

  include/fusion_tree/services/confirm_service.cpp
  include/fusion_tree/services/confirm_target_service.cpp
  include/fusion_tree/services/feedback_service.cpp
  include/fusion_tree/services/get_task_tools_service.cpp
  include/fusion_tree/services/gripper_service.cpp
  include/fusion_tree/services/is_fault_service.cpp 
  include/fusion_tree/services/is_object_detected_service.cpp
  include/fusion_tree/services/list_targets_service.cpp
  include/fusion_tree/services/list_tools_service.cpp
  include/fusion_tree/services/reset_progress_service.cpp
  include/fusion_tree/services/snap_to_tool_service.cpp
  include/fusion_tree/services/stop_service.cpp
  include/fusion_tree/services/task_inspection_service.cpp
  include/fusion_tree/services/task_instructions_service.cpp
  include/fusion_tree/services/task_progress_service.cpp
  include/fusion_tree/services/verify_task_service.cpp

  include/fusion_tree/actions/give_action_client.cpp
  include/fusion_tree/actions/move_action_client.cpp
  include/fusion_tree/actions/pick_action_client.cpp
  include/fusion_tree/actions/place_action_client.cpp
  include/fusion_tree/actions/snap_action_client.cpp

  include/fusion_tree/conditions/is_grasping.cpp
  include/fusion_tree/conditions/is_locked.cpp
  include/fusion_tree/conditions/is_command.cpp
  include/fusion_tree/conditions/is_task_done.cpp
  include/fusion_tree/conditions/is_assembly_started.cpp
)

add_executable(
  ${PROJECT_NAME}_test_node src/fusion_test.cpp 
  include/fusion_tree/test_class.cpp
)

# add_executable(
# ${PROJECT_NAME}_communication_node
# src/test.cpp 
# src/communication_server.cpp
#)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_main_tree_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME}_communication_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_main_tree_node
  ${behaviortree_cpp_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test_node
 ${catkin_LIBRARIES}
)

## C++ library of fusion_tree, used in gtests
target_link_libraries(${PROJECT_NAME} 
  ${behaviortree_cpp_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
#catkin_add_gtest(${PROJECT_NAME}-test test/test_main_tree.cpp)
#catkin_add_gtest(${PROJECT_NAME}-pick-test test/test_pick_subtree.cpp)
#catkin_add_gtest(${PROJECT_NAME}-place-test test/test_place_subtree.cpp)

#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${behaviortree_cpp_LIBRARIES})
#endif()
#if(TARGET ${PROJECT_NAME}-pick-test)
#  target_link_libraries(${PROJECT_NAME}-pick-test ${PROJECT_NAME} ${behaviortree_cpp_LIBRARIES})
#endif()
#if(TARGET ${PROJECT_NAME}-place-test)
#  target_link_libraries(${PROJECT_NAME}-place-test ${PROJECT_NAME} ${behaviortree_cpp_LIBRARIES})
#endif()



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
