cmake_minimum_required(VERSION 2.8.3)
project(mrdplot)

set(MSG_DEPS

)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  )



set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mrdplot
  CATKIN_DEPENDS roscpp roslib
  DEPENDS Eigen3 
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(mrdplot
   src/mrdplot.cpp
 )

## Declare a cpp library
add_library(logger src/Logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES}
                              )
set_target_properties(logger PROPERTIES COMPILE_FLAGS "-std=c++0x -o3")     
                          
