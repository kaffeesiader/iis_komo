cmake_minimum_required(VERSION 2.8.3)
project(iis_komo)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   AddPrimitiveShape.msg
#)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Move.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
)


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
#  LIBRARIES iis_komo
#  CATKIN_DEPENDS roscpp
#  DEPENDS $ENV{KOMO_PATH}/src
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{KOMO_PATH}/src
)

link_directories(
  $ENV{KOMO_PATH}/lib
)

## Declare a cpp library
add_library(komo_wrapper src/komo_wrapper.cpp)
target_link_libraries(komo_wrapper Ors Core Motion ${catkin_LIBRARIES})
set_target_properties(komo_wrapper PROPERTIES COMPILE_FLAGS -std=c++0x)

add_library(robot_interface src/robot_interface.cpp)
target_link_libraries(robot_interface ${catkin_LIBRARIES})
set_target_properties(robot_interface PROPERTIES COMPILE_FLAGS -std=c++0x)

## Declare a cpp executable
add_executable(iis_komo src/iis_komo.cpp)
add_dependencies(iis_komo komo_wrapper robot_interface iis_komo_generate_messages_cpp)
target_link_libraries(iis_komo komo_wrapper robot_interface ${catkin_LIBRARIES})
set_target_properties(iis_komo PROPERTIES COMPILE_FLAGS -std=c++0x)

#add_executable(komo_easy src/main.cpp)
#add_dependencies(komo_easy komo_interface)
#target_link_libraries(komo_easy komo_interface ${catkin_LIBRARIES})
#set_target_properties(komo_easy PROPERTIES COMPILE_FLAGS -std=c++0x)

add_executable(komo_test src/test.cpp)
target_link_libraries(komo_test Core ${catkin_LIBRARIES})
set_target_properties(komo_test PROPERTIES COMPILE_FLAGS -std=c++0x)

add_executable(komo_info src/info.cpp)
target_link_libraries(komo_info Ors Core Motion ${catkin_LIBRARIES})
set_target_properties(komo_info PROPERTIES COMPILE_FLAGS -std=c++0x)
