# clmclogger
clmclogger tool for variable logging (C++) and data manipulation/visualization (MATLAB)

## Description 

clmclogger is a logging tool first developed by Stefan Schaal along with the SL robot simulator. Since then many researchers have modified the code with improvements, new features, etc... But due to the lack of a central repository, each different group developed their own variations of the code.

The goal of this repository is to 1) design/maintain the general C++ logger to be used in many different projects, 2) try integrating some of these different variations of clmclogger into a single common place, and 3) further develop existing functionality.

The current version of C++ wrapper compiles easily as a ROS Catkin project.


#### clmcplot tool

![CLMCPLOT](/docs//clmcplot.png?raw=true "clmcplot")

<!---
#### mrdplot tool (older version)

![MRDPLOT](/docs//mrdplot.png?raw=true "mrdplot")
-->

## C++ Logger Installation

### CATKIN (ROS) Workspace

Clone the software on your catkin workspace source folder:
```bash
cd ~/catkin_ws/src
git clone git@github.com:felipepolido/mrdplot.git
```
And compile the workspace:
```bash
cd ~/catkin_ws/
catkin_make
```

### CMake

Clone the software on your computer:
```bash
cd ~/
git clone git@github.com:felipepolido/clmclogger.git
git submodule update --init
```

Compile:

```bash
mkdir build
cd build
cmake ..
make -j
```

## C++ Logger Examples:

### ROS Examples:

To launch the standalone [example logger](https://github.com/felipepolido/clmclogger/blob/master/example/ExampleLogger.cpp):
```bash
rosrun clmclogger example_logger
```

To launch the ros example, start the 
[ros publisher](https://github.com/felipepolido/clmclogger/blob/master/example/RosExamplePublisher.cpp):
```bash
rosrun clmclogger ros_example_pub
```

And in another terminal start the 
[ros logger](https://github.com/felipepolido/clmclogger/blob/master/example/RosExampleLogger.cpp):
```bash
rosrun clmclogger ros_example_logger
```

### CMake Example:

To launch the standalone [example logger](https://github.com/felipepolido/clmclogger/blob/master/example/ExampleLogger.cpp) in CMake:
```bash
./bin/example_logger
```


## Data Visualization:

First, open MATLAB and include /clmclogger/matlab to MATLAB's path:
- traverse to the clmclogger folder
- right click on the clmclogger/matlab folder
- go to "Add to Path"  
- click on "Select Folders and Subfolders" 

Then on the command window type "clmcplot" or "clmcplot" to open either tool.
At this point open a log by cliking the "Open" button on the GUI.

After the log has opened, to add signals to each plot click 
on the signal and then click on the desired plot.

## Dependencies

To include clmclogger to your ROS project, 
first make sure the CMakeLists.txt find the clmclogger package:

```bash
find_package(catkin REQUIRED
  clmclogger
  )
```

And add the dependency to your package.xml:

```bash
  <build_depend>clmclogger</build_depend>
  <run_depend>clmclogger</run_depend>
```



## Thank you 
Thank you Stefan Schaal for developing clmcplot along with the SL simulator.

Thank you for anyone who has contributed to this library along the way. If you would like to be explicitly aknowledged please let me know or create a pull request.

Thank you personally to:

[Stefan Schaal](http://www-clmc.usc.edu/~sschaal/) ,
[Siyuan Feng](https://github.com/siyuanfeng) ,
[Matt DeDonato](https://github.com/mdedonato) , 
[Chris Atkseon](https://github.com/cga-cmu)
