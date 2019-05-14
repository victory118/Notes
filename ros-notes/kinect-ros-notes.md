## Setup

Install the **freenect_launch** ROS package, which will install the **freenect** node and its dependencies: 

```
sudo apt-get install ros-kinetic-freenect-launch
```

Install the **depthimage_to_laserscan** ROS package:

```
sudo apt-get install ros-kinetic-depthimage-to-laserscan
```

## Launching

Run the example freenect launch file:

```
roslaunch freenect_launch freenect.launch
```

I copied the contents of freenect.launch into my own package:

```
roslaunch my_kinetic_launch freenect.launch
```

See the output on RViz with the **kinect_view.rviz** configuration.

The freenect.launch file will write to topics called **/camera/depth/points** and **/camera/depth_registered/points** (among many other topics), which can be visualized through RViz as messages of type **sensor_msgs/PointCloud2**.

Point cloud data must be converted to a message of type **sensor_msgs/LaserScan** in order to use the mapping packages. To apply this conversion, run the launch file:
```
roslaunch my_kinect_launch depthimage_to_laserscan.launch
```

This launch file will publish the laser scan data to the **/depth_scan** topic. This laser scan data can be visualized in RViz using the **kinect_view-2.rviz** configuration file.

## Controlling the tilt motor and reading the IMU

The ROS package to control the tilt motor angle and read the IMU measurements is called **kinect_aux** and is officially released for the ROS Indigo distribution. However, I tested it in ROS Kinetic and it works! The instructions from the official wiki page <http://wiki.ros.org/kinect_aux> will be repeated here.

Firstly, install the system dependencies (if not yet installed):

```
$ sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev
```

Next, download the source code to your catkin workspace and build it:

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/muhrix/kinect_aux.git -b indigo
$ cd ~/catkin_ws
$ catkin_make
```

Lastly, run the node (remember to run roscore if it is not yet running):

```
$ rosrun kinect_aux kinect_aux_node
```

To set the the tilt angle to, e.g., -15 degrees (Kinect facing downwards), simply publish a message on the topic (see API below):

```
$ rostopic pub /tilt_angle std_msgs/Float64 -- -15
```

To view current tilt angle, run:

```
$ rostopic echo /cur_tilt_angle
```

The tilt angle can be set between -31 and 31 degrees, where -31 degrees is facing down and 31 degrees is facing up.



## References

* http://wiki.ros.org/freenect_launch
* <http://wiki.ros.org/depthimage_to_laserscan>
* <https://www.youtube.com/watch?v=_QpNMJEAkX0> - How to connect Kinect to Raspberry Pi 2 or 3 with ROS
  * <http://roboticsweekends.blogspot.com/2017/12/how-to-connect-kinect-to-raspberry-pi-2.html>
* <http://wiki.ros.org/kinect_aux>
* <https://stackoverflow.com/questions/7696436/precision-of-the-kinect-depth-camera>
* <https://answers.ros.org/question/33091/doubts-about-interpreting-xyz-from-kinect/>
* 