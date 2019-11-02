# example_ros_cpp
An example of a ROS C++ package conforming to the [LARICS C++ coding standard](http://larics.rasip.fer.hr/farm/laricswiki/doku.php?id=software:coding_standard#c_coding_standards) and [ROS developers guide](http://wiki.ros.org/DevelopersGuide).

In this example a PID control algorithm is implemented.

## Dependencies

The code has been developed and tested on a Ubuntu 16.04 system with ROS Kinetic. The following libraries are required to build the code and developer docs:

 * Robot Operating System
 * doxygen

## ROS package description

### ROS dependencies

 * [std_msgs](http://wiki.ros.org/std_msgs)

### Nodes
 * pid_controller_node
 
   The node implements a PID controller algorithm. It receives the setpoint and a measurement of the process variable through the appropriate ROS topics and publishes the output, the controlled variable.

### Subscribed topics
 * setpoint ([std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html))
 * measurement ([std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html))

### Published topics
 * output ([std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html))

## Quickstart

### Building the code

The code is built in a standard ROS fashion. In your ROS workspace root, execute the following command:

 ```
 catkin build example_ros_cpp
 ```

### Building and viewing the docs

From the package root (where `Doxyfile` is located), run:

```
doxygen
firefox doc/html/index.html &
```
