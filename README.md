# clmcplot
clmcplot tool for variable logging (C++) and data manipulation/visualization (MATLAB)

## Description 

mrdplot/clmcplot is a logging tool first developed by Stefan Schaal along with the SL robot simulator. Since then many researchers have modified the code dramatically with improvements, new features, etc... But unfortunately, due to the lack of a central repository, each different group developed their own variations of the code.

The goal of this repository is to 1) try integrating some of these different variations into a single common place, 2) further develop existing functionality.

The current version of C++ wrapper compiles easily as a ROS Catkin project.


#### clmcplot tool

![CLMCPLOT](/docs//clmcplot.png?raw=true "clmcplot")

#### mrdplot tool (older version)

![MRDPLOT](/docs//mrdplot.png?raw=true "mrdplot")


## How-to use (ROS environment)


### Installation

Clone this package on your catkin workspace source folder:
```bash
cd ~/catkin_ws/src
git clone git@github.com:felipepolido/mrdplot.git
```
And compile the workspace:
```bash
cd ~/catkin_ws/
catkin_make
```

As of this writing the log files will be saved in the current folder by default.


### Example:
To launch the [example logger](https://github.com/felipepolido/mrdplot/blob/master/example/ExampleLogger.cpp):
```bash
rosrun clmcplot example_logger
```

### ROS Example:

To launch the ros example, start the 
[ros publisher](https://github.com/felipepolido/mrdplot/blob/master/example/RosExamplePublisher.cpp):
```bash
rosrun clmcplot ros_example_pub
```

And in another terminal start the 
[ros logger](https://github.com/felipepolido/mrdplot/blob/master/example/RosExampleLogger.cpp):
```bash
rosrun clmcplot ros_example_logger
```

### Data Visualization:

First, open MATLAB and include /clmcplot/matlab to MATLAB's path:
- traverse to the clmcplot folder
- right click on the clmcplot/matlab folder
- go to "Add to Path"  
- click on "Select Folders and Subfolders" 

Then on the command window type "clmcplot" or "clmcplot" to open either tool.
At this point open a log by cliking the "Open" button on the GUI.

After the log has opened, to add signals to each plot click 
on the signal and then click on the desired plot.

### Dependencies

To include clmcplot to your project, 
first make sure the CMakeLists.txt find the clmcplot package:

```bash
find_package(catkin REQUIRED
  clmcplot
  )
```

And add the dependency to your package.xml:

```bash
  <build_depend>clmcplot</build_depend>
  <run_depend>clmcplot</run_depend>
```



## Thank you 
Thank you Stefan Schaal for developing clmcplot along with the SL simulator.

Thank you for anyone who has contributed to this library along the way. If you would like to be explicitly aknowledged please let me know or create a pull request.

Thank you personally to:

[Stefan Schaal](http://www-clmc.usc.edu/~sschaal/) ,
[Siyuan Feng](https://github.com/siyuanfeng) ,
[Matt DeDonato](https://github.com/mdedonato) , 
[Chris Atkseon](https://github.com/cga-cmu)
