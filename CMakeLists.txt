cmake_minimum_required(VERSION 2.8.3)
project(edo_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rosbridge_library
  rosbridge_server
  roscpp
  rosserial_msgs
  rosserial_client
  std_msgs
  edo_core_msgs
  roslib
  cmake_modules
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/orl/include/")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES edo_core_pkg
   CATKIN_DEPENDS  roscpp  std_msgs edo_core_msgs
   DEPENDS TinyXML
)

###########
## Build ##
###########

set( SOURCE_STATE_MACHINE src/edo_state_machine_node.cpp
						  src/StateManager.cpp
						  src/CalibrateState.cpp
						  src/NotCalibrateState.cpp
						  src/State.cpp
						  src/SubscribePublish.cpp
						  src/CommandState.cpp
						  src/InitState.cpp
						  src/CommonService.cpp
						  src/JogState.cpp
						  src/MoveCommandState.cpp
						  src/ErrorState.cpp
						  src/BrakeState.cpp)

# edo_recovery node
add_executable(edo_recovery src/edo_recovery_node.cpp )
add_dependencies(edo_recovery ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_recovery ${catkin_LIBRARIES})

# edo_state_machine node
add_executable(edo_state_machine ${SOURCE_STATE_MACHINE} )
add_dependencies(edo_state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_state_machine ${catkin_LIBRARIES})
target_link_libraries(edo_state_machine ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# edo_jnt_handler node
add_executable(edo_jnt_handler src/edo_jnt_handler_node.cpp)
add_dependencies(edo_jnt_handler ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_jnt_handler ${catkin_LIBRARIES})

# edo_algorithms node
add_executable(edo_algorithms src/edo_algorithms_node.cpp src/AlgorithmManager.cpp)
add_dependencies(edo_algorithms ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_algorithms ${catkin_LIBRARIES})
target_link_libraries(edo_algorithms ${catkin_LIBRARIES} "/opt/orl/lib/libeORL.so")

# edo_bridge node
add_executable(edo_bridge src/edo_bridge_node.cpp)
add_dependencies(edo_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_bridge ${catkin_LIBRARIES})

# edo_usb_fake node
add_executable(edo_usb_fake src/edo_usb_fake_node.cpp)
add_dependencies(edo_usb_fake ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_usb_fake ${catkin_LIBRARIES})

# edo_c5g node
add_executable(edo_c5g src/edo_c5g_node.cpp src/C5GManager.cpp)
add_dependencies(edo_c5g ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_c5g ${catkin_LIBRARIES})

# edo_moveit_fake node
add_executable(edo_moveit_fake src/edo_moveit_fake_node.cpp)
add_dependencies(edo_moveit_fake ${catkin_EXPORTED_TARGETS})
target_link_libraries(edo_moveit_fake ${catkin_LIBRARIES})

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(NODE_INCLUDE "include/")
include_directories(
# include
  ${NODE_INCLUDE}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/edo_core_pkg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/edo_core_pkg_node.cpp)

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
# install(TARGETS ${PROJECT_NAME} edo_algorithms
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_edo_core_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
