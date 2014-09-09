cmake_minimum_required(VERSION 2.8.3)
project(iis_komo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
# add_library(iis_komo
#   src/${PROJECT_NAME}/iis_komo.cpp
# )

## Declare a cpp executable
add_executable(komo_easy src/main.cpp)
target_link_libraries(komo_easy Ors Core Motion ${catkin_LIBRARIES})
set_target_properties(komo_easy PROPERTIES COMPILE_FLAGS -std=c++0x)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(iis_komo_node iis_komo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(iis_komo_node
#   ${catkin_LIBRARIES}
# )
