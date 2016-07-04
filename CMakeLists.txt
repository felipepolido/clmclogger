cmake_minimum_required(VERSION 2.8.3)
project(clmcplot)

set(MSG_DEPS

)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g ${CMAKE_CXX_FLAGS}")

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
add_library(mrdplot src/mrdplot.cpp)

add_library(MRDLogger src/MRDLogger.cpp)
target_link_libraries(MRDLogger mrdplot)
set_target_properties(MRDLogger PROPERTIES COMPILE_FLAGS "-std=c++0x -g -o3")     


## Declare a cpp library
add_library(logger src/Logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES}
                             MRDLogger )
set_target_properties(logger PROPERTIES COMPILE_FLAGS "-std=c++0x -g -o3")     
        
add_executable(testMRD test/test.cpp)
target_link_libraries(testMRD MRDLogger)

add_executable(example_logger example/ExampleLogger.cpp)
target_link_libraries(example_logger  MRDLogger ${catkin_LIBRARIES})

add_executable(ros_example_logger example/RosExampleLogger.cpp)
target_link_libraries(ros_example_logger  logger ${catkin_LIBRARIES})

add_executable(ros_example_pub example/RosExamplePublisher.cpp)
target_link_libraries(ros_example_pub ${catkin_LIBRARIES})

                          


