## Todos

1. Broadcast the tag poses in the world frame in cam2tag_broadcaster.py.
   1. Find some way to load them using a .yaml file. You may have to manually find the quaternion rotation matrix based on the 3x3 rotation matrix which is easy to find. All ceiling tags will have the same relative rotation in the world frame.
2. Broadcast 



## Hardware connections

Digital Pins Usable for Interrupts: 2, 3, 18, 19, 20, 21 (attachInterrupt: 0, 1, 5, 4, 3, 2)

| Arduino Mega             | Connection point                               |
| ------------------------ | ---------------------------------------------- |
| D19 (external interrupt) | LS7184 #3 (from top): pin 8 (clock out)/fleft  |
| D23                      | LS7184 #3 (from top): pin 7 (direction)/fleft  |
| D18 (external interrupt) | LS7184 #2 (from top): pin 8 (clock out)/fright |
| D22                      | LS7184 #2 (from top): pin 7 (direction)/fright |
| D2 (external interrupt)  | LS7184 #1 (from top): pin 8 (clock out)/rright |
| D25                      | LS7184 #1 (from top): pin 7 (direction)/rright |
| D3 (external interrupt)  | LS7184 #4 (from top): pin 8 (clock out)/rleft  |
| D24                      | LS7184 #4 (from top): pin 7 (direction)/rleft  |
| D4 (PWM)                 | Front left motor (direction)                   |
| D5 (PWM)                 | Front left motor (PWM)                         |
| D6 (PWM)                 | Front right motor (direction)                  |
| D7 (PWM)                 | Front right motor (PWM)                        |
| D8 (PWM)                 | Rear right motor (direction)                   |
| D9 (PWM)                 | Rear right motor (PWM)                         |
| D10 (PWM)                | Rear left motor (direction)                    |
| D11 (PWM)                | Rear left motor (PWM)                          |

| LS7184 Encoder Counter      | Connection point            |
| --------------------------- | --------------------------- |
| LS7184 #3 (from top): pin 4 | Front left wheel encoder A  |
| LS7184 #3 (from top): pin 5 | Front left wheel encoder B  |
| LS7184 #2 (from top): pin 4 | Front right wheel encoder A |
| LS7184 #2 (from top): pin 5 | Front right wheel encoder B |
| LS7184 #1 (from top): pin 4 | Rear right wheel encoder A  |
| LS7184 #1 (from top): pin 5 | Rear right wheel encoder B  |
| LS7184 #4 (from top): pin 4 | Rear left wheel encoder A   |
| LS7184 #4 (from top): pin 5 | Rear left wheel encoder B   |
|                             |                             |

## Transferring files from laptop to RPi

You are SSHed into the RPi from your laptop and your want to transfer some files from the laptop onto the RPi. To do this, navigate to the directory where the file or folder exists on the laptop and run the command:

```
$ scp -r farmaid_ros victor@192.168.1.11:/home/victor/sketchbook
```

Here, `farmaid_ros` is the entire directory to be copied from the laptop to the RPi. The argument `-r` is the recursive option to use when you want to copy an entire directory as opposed to just a single file. The directory on the RPi where you want to copy the file to is `/home/victor/sketchbook`.

## Setting up the ROS environment

Open a new terminal. If the Anaconda environment is activated, deactivate it by running `conda deactivate`.

Source the ROS catkin workspace by running:

```
$ cd ~/path_to_catkin_ws/
$ source devel/setup.bash
```



## USB Camera and AprilTags

Launch the USB camera node and AprilTag detection node by running:

```
$ roslaunch robot_launch robot.launch
```

The `robot.launch` file launches two other launch files: `camera_rect.launch` and `tag_detection.launch`. `camera_rect.launch` starts the USB camera node. In this file is where you can set the appropriate device name. For example, if I want to use the USB webcam I would set the device name as "/dev/video1". If I want to use the integrated webcam, the device name would be "/dev/video0". `tag_detection.launch` starts the AprilTag detector node. In this file is where you can set the size of each AprilTag corresponding to its ID number.

After launching the above, you can view the AprilTags being detected while the webcam is streaming by opening another terminal and running `rqt_image_view`. In the drop-down menu, select "/camera/tag_detections_image" and hold the AprilTag in front of the camera to verify that it gets detected.

To see the pose of the AprilTags detected with respect to the camera, open another terminal and run `rostopic echo /camera/tag_detections_pose`.

## Converting AprilTag measurements to robot coordinate frame

The AprilTag detector node publishes pose information to the "topic_name" topic. This is the pose of the tag with respect to the camera coordinate frame. The ultimate goal is to get the pose of the tag in the robot coordinate frame.

Before deriving the final results, I plan to study the RosInterface class that was provided by the Robotics Capstone course from Coursera in hopes of reusing as much of their code as possible. A brief description of their code is provided below.

## Publishing AprilTag static transform

The AprilTags will have a fixed position and orientation in the world map. Thus, we will have to publish their pose information with respect to the world coordinate frame. One way to publish a static transform is through a launch file. Create a new launch file and use the code below as an example:

```
<launch>
        <node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="0.1 0 0.455 0 0 0 base_link camera_link 100"/>
</launch>
```

## Making a tf broadcaster

Create a new Python file called **cam2tag_broadcaster.py** and copy the code below.

```python
#!/usr/bin/env python
import rospy

import tf
import tf2_ros
import geometry_msgs.msg

from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection,
)


def tag_pose_callback(pose_array):

    if (len(pose_array.detections)==0):
        return

    for i in range(len(pose_array.detections)):
        pose = pose_array.detections[i].pose.pose
        tag_tf = geometry_msgs.msg.TransformStamped()
        tag_tf.header.stamp = rospy.Time.now()
        tag_tf.header.frame_id = "camera"
        tag_tf.child_frame_id = "tag" + str(i)
        tag_tf.transform.translation.x = pose.position.x
        tag_tf.transform.translation.y = pose.position.y
        tag_tf.transform.translation.z = pose.position.z
        tag_tf.transform.rotation.x = pose.orientation.x
        tag_tf.transform.rotation.y = pose.orientation.y
        tag_tf.transform.rotation.z = pose.orientation.z
        tag_tf.transform.rotation.w = pose.orientation.w
        br.sendTransform(tag_tf)

if __name__ == '__main__':
    rospy.init_node('cam2tag_broadcaster')

    sub = rospy.Subscriber("/camera/tag_detections",AprilTagDetectionArray,tag_pose_callback)
    br = tf2_ros.TransformBroadcaster()

    rospy.spin()

```

Be sure to make the Python file an executable by running `chmod +x cam2tag_broadcaster.py`.

## Converting from rotation matrix to quaternion

The standard format for passing transforms in ROS is using quaternions. However, it is usually more intuitive to derive transforms in the form of a rotation matrix first and then convert it into quaternion using software. A helpful tool for doing these conversions is here: <https://www.andre-gaschler.com/rotationconverter/>

## Using ROS fiducials package

The official link is <http://wiki.ros.org/fiducials> and another helpful link is <https://learn.ubiquityrobotics.com/fiducials>.

If you need to install the **cairosvg** module, run `sudo apt-get install python-cairosvg`.

The **fiducials** package is a container for two packages: **aruco_detect** and **fiducial_slam**.

* <http://wiki.ros.org/aruco_detect>

* <http://wiki.ros.org/fiducial_slam>

The install the fiducials package from binary packages:
```
sudo apt-get install ros-kinetic-fiducials
```

Fiducial markers can be generated with a command like this:
```
rosrun aruco_detect create_markers.py 100 112 fiducials.pdf
```
Once printed, they can be affixed to the environment. They don't need to be placed in any particular pattern but the density should be such that two or more markers can be seen by the camera on the robot, so the map can be built. Placing them on the ceiling reduces problems with occlusion, but is not required, since a full 6DOF pose is estimated.

Two nodes should be run, aruco_detect, which handles the detection of the fiducials, and fiducial_slam, which combines the fiducial pose estimates and builds the map and makes an estimate of the robot's position. The map is in the form of a text file specifying the 6DOF pose of each of the markers, and is automatically saved.

There are launch files for both of these nodes:
```
roslaunch aruco_detect aruco_detect.launch
roslaunch fiducial_slam fiducial_slam.launch
```
A launch file is also provided to visualize the map in rviz.
```
roslaunch fiducial_slam fiducial_rviz.launch
```

You need to run the camera node before running the above launch files. You will also have to modify the launch files `aruco_detect.launch` and `fiducial_slam.launch` so that they have the correct parameters for the camera name, image name, etc. For example, the launch file below runs the camera node,  `aruco_detect.launch` and `fiducial_slam.launch` files at the same time with custom parameters:
``` xml
<launch>
  <arg name="camera" default="usb_cam"/>
  <arg name="image" default="image_raw"/>

  <include file="$(find robot_launch)/launch/camera_rect.launch"/>
  <include file="$(find aruco_detect)/launch/aruco_detect.launch">
    <arg name="camera" value="$(arg camera)" />
    <arg name="image" value="$(arg image)"/>
  </include>
  <include file="$(find fiducial_slam)/launch/fiducial_slam.launch">
    <arg name="camera" value="$(arg camera)" />
    <arg name="publish_6dof_pose" value="true"/>
  </include>

  <node pkg="tf" type="static_transform_publisher" name="camera_base"
    args="0.105 0.0 0.155 0.0 0.0 0.7071068 0.7071068 base_link usb_cam 50"/>
  <node pkg="tf" type="static_transform_publisher" name="base_odom"
    args="0.0 0.0 0.0 0.0 0.0 0.0 1.0 odom base_link 50"/>
  <!-- <node pkg="tf2_ros" type="static_transform_publisher" name="camera_base"
    args="0.105 0.0 0.155 0.0 0.0 -1.5707963 base_link usb_cam"/> -->
  <!-- <node pkg="tf" type="static_transform_publisher" name="tag1_broadcaster" args="1 0 0 0 0 0 1 world tag1 100" /> -->
</launch>

```

The transform from the **base_link** to **usb_cam** should be handled by the static_transform_publisher because there is no relative motion between these two frames so this transform is static. The pose of frame **usb_cam** in frame **base_link** is easy to obtain using a ruler and by inspection because the all of the coordinate axes in both frames are orthogonal.

In the above launch file, the transform from **odom** to **base_link** transform is also statically published and the pose of **base_link** frame in **odom** frame is set to zero (i.e., the coordinate frames are exactly aligned). Hence, the pose of the **odom** and **base_link** frames in the **map** (or world) frame are equivalent and they only depend on the fiducial marker detections. The correct way to set this transform is to update it using encoder/IMU/gyro information in every time step. For example, assume in the previous time step that you have an estimate of the robot's pose (i.e., the pose of **base_link** frame in **odom** frame). In the current time step, you use measurements from the the encoder/IMU/gyro to calculate a new estimate of the robot pose and broadcast this transform as **odom** frame to **base_link** frame. In the same current time step, the robot's camera detects a fiducial marker which has a known pose in the **map** frame. Recall that the goal is to estimate the pose of **base_link** frame in the **map** frame, $^{\text{map}}T_{\text{base}}$, and we know $^{\text{cam}}T_{\text{fid}}$, $^{\text{odom}}T_{\text{base}}$, $^{\text{base}}T_{\text{cam}}$, and $^{\text{map}}T_{\text{fid}}$. $^{\text{map}}T_{\text{base}}$ can be solved by chaining together the known transforms as $^{\text{map}}T_{\text{base}} = ^{\text{map}}T_{\text{fid}} \cdot (^{\text{cam}}T_{\text{fid}})^{-1} \cdot (^{\text{base}}T_{\text{cam}})^{-1}$. $^{\text{map}}T_{\text{odom}}$ also gets updated as $^{\text{map}}T_{\text{odom}} = ^{\text{map}}T_{\text{base}} \cdot (^{\text{odom}}T_{\text{base}})^{-1}$. The transform from **map** frame to **odom** frame provides an estimate of the accumulated error/drift in robot's pose that should be accounted for when estimating the pose of **base_link** frame in the **map** frame. Hence, the corrected pose of **base_link** in **map** frame can be obtained by $^{\text{map}}T_{\text{base}} = ^{\text{map}}T_{\text{odom}} \cdot ^{\text{odom}}T_{\text{base}}$.

If we assume that the **odom** and **base_link** frames are the same and static relative to one another, i.e., we do not update the transform using odometry measurements, then we can simply use the estimated pose of **odom** or **base_link** as the pose in the **map** frame. Later, I will try fusing this pose estimate with odometry information using a Kalman filter to see if I can obtain a smoother final pose estimate. Here are some links describing the **base_link**, **odom**, and **map** frames in more detail:
1. http://www.ros.org/reps/rep-0105.html.
2. http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom. 
3. <https://answers.ros.org/question/44639/how-to-broadcast-a-transform-between-map-and-odom/>
4. http://library.isr.ist.utl.pt/docs/roswiki/hector_slam(2f)Tutorials(2f)SettingUpForYourRobot.html

## Testing Teleop Mode with roscore on Ubuntu laptop and Arduino code on RPi

First export the IP addresses on the Ubuntu laptop and RPi so they can find each other on the ROS server. The Ubuntu laptop will be running roscore. Hence on the RPi, you should declare the ROS master IP address by running:

```
$ export ROS_MASTER_URI=http://192.168.1.7:11311/
```
where 192.168.1.7 is the IP address of the Ubuntu laptop running roscore. Because the IP address can be different, you find it by running `hostname -I`. The RPi will also have to export its own IP address to the ROS server by running:
```
$ export ROS_IP=192.168.1.11
```
These ROS variables can be verified by running `echo $ROS_MASTER_URI` and `echo $ROS_IP`.

Now go to the Ubuntu laptop and export its IP address to the ROS server by running:
```
$ export ROS_IP=192.168.1.7
```

Note that whenever open a new terminal to run a ROS node on the Ubuntu laptop or RPi, you will have to export the IP addresses again as explained above.

Open a new terminal on the Ubuntu laptop and run `roscore`.

Now open a new terminal on the RPi and run the serial node:
```
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```
Next open a new terminal on the Ubuntu laptop and run the `teleop_twist_keyboard` node, which will allow you to publish velocity commands to the `/cmd_vel` topic using the keyboard to send commands:

```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Alternatively, use the turtlebot_teleop_key which might work better because it stops the robot when the keyboard is not pressed:
```
$ rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/cmd_vel
```

If you are uploading code to the Arduino Mega instead of the Arduino Uno, you need to change to the correct setting in the Makefile. Go to the Makefile and change the line from:

```
BOARD_TAG = uno
```

to:

```
BOARD_TAG = mega2560
```

If the board is different from the Uno or Mega, you can see all supported boards by running in the command line `make show_boards`.

### Running the fiducials SLAM package

Open a terminal in the Ubuntu laptop and SSH into the RPi:
```
$ ssh victor@192.168.1.1.7
```

In the SSH RPi terminal, pull the latest changes of the farmaid_bot ROS package from GitHub:
```
$ git pull
```
Verify that you have exported the ROS_IP of the RPi and set the ROS_MASTER_URI to the IP address of the Ubuntu laptop.

Plug the USB camera into the RPi and check the name of the device:
```
$ ls -ltrh /dev/video*
```
In my case the USB camera was named **/dev/video0**. Go to the camera_rect.launch file and make sure that the name of the device is set to the above name.

Open a new terminal in the Ubuntu laptop a verify that you have exported the ROS_IP of the Ubuntu laptop. Then run `roscore`.


## Mount Raspbian image file in Ubuntu

I have a .img file that contains the Raspbian OS and some other files. This .img file is meant to be flashed onto the Raspberry Pi. However, I only want to access some of the files on this .img file. How do I access the contents of the .img file on my Ubuntu OS?

Check the file system table with fdisk:

```
$ fdisk -l coursera_robotics_capstone_pi.img
Disk coursera_robotics_capstone_pi.img: 6.5 GiB, 6920601600 bytes, 13516800 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x54cef515

Device                             Boot  Start      End  Sectors  Size Id Type
coursera_robotics_capstone_pi.img1        8192   131071   122880   60M  c W95 FAT32 (LBA)
coursera_robotics_capstone_pi.img2      131072 13443071 13312000  6.4G 83 Linux
```

The important parts from the above output are:

* Sector size: 512
* Start blocks for img1 (8192) and img2 (131072)

Now create a directory mount point for both images:

```
mkdir img1 img2
```

When your mount point directories are ready, mount both images with the sector size and start block information you have gathered in previous step:

```
$ sudo mount coursera_robotics_capstone_pi.img -o loop,offset=$((512*8192)) img1/
$ sudo mount coursera_robotics_capstone_pi.img -o loop,offset=$((512*131072)) img2/
```

The contents are now contained in the `img1/` and `img2/` folders.

### References

1. https://linuxconfig.org/how-to-mount-rasberry-pi-filesystem-image

## Creating a catkin package

Navigate to the `src` folder of your catkin workspace and create the package with the name and dependencies as arguments:

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg farmaid_bot std_msgs rospy roscpp
```

The general form is shown below.

```
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

To build a catkin package in the catkin workspace:

```
$ cd ~/catkin-ws
$ catkin_make
```

To add the workspace to your ROS environment you need to source the generated setup file:

```
$ . ~/catkin_ws/devel/setup.bash
```

### References

1. http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage
2. 






