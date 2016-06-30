cmake_minimum_required(VERSION 2.8.3)
project(mrdplot)

set(MSG_DEPS

)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mrdplot logger
  CATKIN_DEPENDS roscpp roslib
  DEPENDS Eigen3 
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(mrdplot
   src/mrdplot.cpp
 )

## Declare a cpp library
add_library(logger src/Logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES}
                             mrdplot )
set_target_properties(logger PROPERTIES COMPILE_FLAGS "-std=c++0x -o3")     
        
add_executable(example_logger src/ExampleLogger.cpp)
target_link_libraries(example_logger  logger ${catkin_LIBRARIES})

add_executable(ros_example_logger src/RosExampleLogger.cpp)
target_link_libraries(ros_example_logger  logger ${catkin_LIBRARIES})

add_executable(ros_example_pub src/RosExamplePublisher.cpp)
target_link_libraries(ros_example_pub ${catkin_LIBRARIES})
