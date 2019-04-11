
Line-following Robot - Documentation

Line-following Robot - Documentation



## High level ideas

- Use ROS framework
- Use similar software architecture as Coursera: Control of Mobile Robots
- Use Arduino to take motor encoder measurements. Publish current measured motor speed at lower sample rate compared to motor speed controller sample rate.
- Design a PID motor speed controller internal to Arduino program that operates at a higher sample rate than it gets published back to RPi for debug purposes.
- At first, just use a Proportional § controller to control velocity.

## Requirements

- Publisher node in Arduino to publish current measured motor speed at 20 Hz
- Subscriber node in Arduino to get desired motor speed
- Robot and motor parameters will be initialized in the main RPi program and this will convert a desired linear velocity to motor speed commands for the Arduino.

## To-dos:

1. Create new local and remote git repo for project.
2. Create and build new ROS workspace in Ubuntu laptop. You will SCP the entire workspace into RPi after finishing development on the Ubuntu laptop.
3. Create new Python node in RPi that publishes speed command that Arduino node subscribes to.

## 1. Hardware setup

### 1.1 Motor and Motor Driver

I am using a motor driver that I bought from eBay. Connect the Arduino to the motor driver input. Update: I suspect the motor driver using the connections below is now broken. Refer to the bottom of this section for information about the new motor driver.

- Arduino: D6 → Motor driver in: PWM1
- Arduino: D7 → Motor driver in: DIR1
- Arduino: D5 → Motor driver in: PWM2
- Arduino: D4 → Motor driver in: DIR2
- Arduino: GND → Motor driver in: GND
- Arduino: 5V → Motor driver in: +5V

Note that pins 5 and 6 on the Arduino Uno are capable of generating PWM signals (denoted by squiggly line next to the pin number), which is the reason we use these pins.

Connect the left motor terminal leads to the the motor driver output (MOTOR1) controlled by PWM1/DIR1. Also connect the right motor terminal leads to the motor driver output (MOTOR2) controlled by PWM2/DIR2.

I am now using the [Adafruit DRV8833](https://learn.adafruit.com/adafruit-drv8833-dc-stepper-motor-driver-breakout-board/pinouts) dual motor driver. The connections are mostly the same as the previous motor driver:

- Arduino: D6 → Motor driver in: AIN1 (check direction)
- Arduino: D7 → Motor driver in: AIN2 (check direction)
- Arduino: D5 → Motor driver in: BIN1 (check direction)
- Arduino: D4 → Motor driver in: BIN2 (check direction)
- Arduino: GND → Motor driver in: VMotor -
- Arduino: 5V → Motor driver in: VMotor + (not VM)

According to the data sheet, the ground on the input and output side are both connected internally. You may need to switch AIN1/AIN2 and BIN1/BIN2 depending on how you want the motors to respond to positive and negative inputs. According to the datasheet, there is no distinction between the PWM and direction (high or low) pins, so you will have to figure out how to connect these by trial and error. Another helpful data sheet for the same type of motor driver is the [Pololu DRV8833 Dual Motor Driver Carrier](https://www.pololu.com/product/2130).

### 1.2 Motor encoder

Connect the [motor encoder](https://drive.google.com/open?id=1kjq3UPpaEjHh_ubWrhepvf_b_dGHPK1R) to the [LS7184 quadrature encoder](https://drive.google.com/open?id=1z6_pIpoR8QxSG-4ZV_2vqcSLydtTcn4L).

- Red (power) to 4.5-7.5V supply (from Arduino 5V)
- Black (ground) to ground terminal (from Arduino GND)
- Blue (Encoder B phase output) to LS7184 pin 5
- Green (Encoder A phase output) to LS7184 pin 4

Connect the LS7184 quadrature encoder to the Arduino. For the LS7184 connected to the left motor encoder:

- LS7184 pin 8 (clock out) → D2 (external interrupt)
- LS7184 pin 7 (direction) → D8

For the LS7184 connected to the right motor encoder:

- LS7184 pin 8 (clock out) → D3 (external interrupt)
- LS7184 pin 7 (direction) → D9

Note that the LS7184 clock out pins must be connected to the [Arduino external interrupt](http://playground.arduino.cc/Code/Interrupts) pins which correspond to pins 2 and 3 on the Arduino Uno.

Finish connecting the remaining LS7184 pins for each motor.

- LS7184 pin 1 (Rbias) to GND (Arduino) via resistor. The instructions recommend a 410k ohm resistor, but I am using a 330k ohm resistor and it seems to work fine.
- LS7184 pin 2 (+VDC) to 5V (Arduino)
- LS7184 pin 3 to GND (Arduino)
- LS7184 pin 6 (Mode input) to GND (Arduino). The different modes are:

1. Mode = 0 VDC (GND) = x1 selection
2. Mode = +VDC (5V) = x2 selection
3. Mode = Float = x4 selection

For example, if I choose Mode = 0 VDC and the encoder outputs 1920 pulses per revolution, then the final clock output from the quadrature encoder will be 1920 x 1 = 1920 pulses per revolution. This seems to be a high enough resolution so I will stick with using the x1 Mode for now.

> Note: I ended up using Mode = +VDC (5V) for both encoders because this gave an output for 1920 pulses per revolution. I checked this by manually rotating the motor and reading the encoder counts from the serial monitor.

### 1.3 Infrared Sensor Array

To detect the line, I will be using the [Sunfounder 8 Channel IR Sensor Array](https://www.amazon.com/SunFounder-Follower-Infrared-Detection-STM8S105C4/dp/B01HB5YVUG). This IR sensor communicates using the I2C (pronounced I-squared-C) serial protocol, which stands for inter-integrated circuit. To read measurements on the Arduino Uno, you will have to make the following connections:

- 5V (sensor) -> 5V (Arduino)
- GND (sensor) -> GND (Arduino)
- SDA (sensor) -> A4 (Arduino)
- SCL (sensor) -> A5 (Arduino)

<https://www.arduino.cc/en/Reference/Wire>

SCL stands for serial clock and is the clock line. SDA stands for serial data and is the data line.

## 2. Software Setup

Create a catkin workspace following this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```
$ cd ~/LineFollowingRobot/linefollow_ws
$ catkin_make
```

Create a new catkin package following this tutorial.

```
$ cd ~/catkin_ws/src  
$ catkin_create_pkg rosbots_driver std_msgs rospy roscpp geometry_msgs
```

## Controlling the Robot Using Teleop

First upload `readEncoder_ros.ino` to the Arduino Uno. Note that `.ino` files are usually stored inside of a directory with the same name. You want to transfer the directory itself containing the `.ino` file.

If the Arduino is connected to your RPi and you are connected to your RPi from your Ubuntu laptop through SSH, then you will first need to transfer the Arduino files from your laptop to the RPi in the proper directory. If `readEncoder_ros` is the name of the Arduino directory, then transfer using scp by running:

```
$ scp readEncoder_ros pi@192.168.1.11:/home/pi/sketchbook
```

To transfer a directory, you need to append the `-r` option after `scp`.
Copy a Makefile into the `readEncoder_ros` directory:

```
$ cd ~/sketchbook  
$ cp Makefile readEncoder_ros/Makefile
```

Add all of the required libraries to the Makefile. In my case, I added `ros_lib` and `TimerOne` to the `ARDUINO_LIBS` setting. The Makefile should look something like this:

```makefile
ARDUINO_DIR = /usr/share/arduino  
ARDUINO_PORT = /dev/ttyACM*  
  
USER_LIB_PATH = /home/pi/sketchbook/libraries  
ARDUINO_LIBS += ros_lib \  
				TimerOne  
BOARD_TAG = uno
```

Also, you have to download or transfer the Arduino libraries from your Ubuntu laptop to your RPi. In my case, I installed the `ros_lib` library using the instructions here without modification:

<http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup>

I transferred the existing Arduino `TimerOne` library from my Ubuntu laptop to the RPi using `scp` to the `~/sketchbook/libraries` directory.

Now to compile and upload the sketch on the Arduino board, open a terminal window in the `readEncoder_ros`directory and run:

```
$ make upload clean
```

Next open a new terminal and start the ROS master:

```
$ roscore  
```

Open a new terminal and start the ROS serial node:

```
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

Open a new terminal and start the `part2_cmr` ROS node:

```
$ rosrun rosbots_driver part2_cmr.py
```

If you haven’t installed the `teleop_twist_keyboard` package, do so by running:

```
$ sudo apt-get install ros-indigo-teleop-twist-keyboard
```

Open a new terminal and start the `teleop_twist_keyboard` ROS node:

```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
```

By default, the teleop_twist_keyboard ROS node publishes Twist messages to a topic named `/cmd_vel`. The `/cmd_vel:=/part2_cmr/cmd_vel` argument tells rosrun to remap the `/cmd_vel` name to `/part2_cmr/cmd_vel`so now the published topic name is the same for both the publishing `teleop_twist_keyboard` ROS node and the subscribing `/part2_cmr` ROS node.

Once you have the `teleop_twist_keyboard` ROS node running, let’s tune the linear and angular speed factors first. Hit the “x” button until you get to around speed 0.174 and hit the “e” key util you get turn 2.59.

As described by the output from running `teleop_twist_keyboard`, use the lower case u i o j k l… keys to drive your ROSbots robot.

## Installing

## Common ROS Commands

### Starting a new catkin workspace

Create a new folder for the catkin workspace and a subdirectory called `src`:

```
$ mkdir -p catkin_ws/src
```

The `-p` flag tells the `mkdir` command to create the main directory first (`catkin_ws`) if it doesn’t already exist. The catkin workspace can be named anything you want. It does not have to be named `catkin_ws`. This is just a common name to refer to the ROS workspace where you will run catkin commands. The source subdirectory should be named `src`.

### Creating a new package

Packages should be created in the `~/catkin_ws/src directory`. For example, to create a new package named beginner_tutorials you would first create the package directory and source subdirectory:

```
$ cd ~/catkin_ws/src
$ mkdir -p beginner_tutorials/src
$ cd ~/catkin_ws/src/beginner_tutorials/src
```

Add/Edit source files.

```
$ cd ~/catkin_ws/src/beginner_tutorials
```

Update `CMakeFiles.txt` in the main package directory to reflect any changes to your sources.

```
$ cd ~/catkin_ws  
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

Run the `catkin_make` command in the main `catkin_ws` directory to build all of the packages. You can also specify exactly which packages you want to be built. For example:

```
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"
```

If you want to revert back to building all packages, do the following:

```
$ catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

For development purposes, you will usually not have to enter in any options when using the `catkin_make`command.

### Sourcing the workspace

In order to run the newly created package, you need to source your workspace so ROS knows where to look for it. To do this, run:

```
$ cd ~/catkin_ws
$ source devel/setup.bash
```

## Building the Line-following Course

Pololu has provided a good tutorial on how to build a line-following course:
<https://www.pololu.com/docs/pdf/0J22/building_line_courses.pdf>

## Control Strategy

The line-following controller will use a proportional-derivative (PD) control architecture to generate an angular velocity command to the robot using the heading angle error measurement as feedback. The reference coordinate system will be aligned tangent to the path and the deviation of the center of the IR sensor array from the path is considered the heading angle error.

### Multi-rate sampling

The wheel velocity control loop runs at 200 Hz. Initially, I ran the line-following controller at the same frequency as the wheel velocity control loop, but I think it would perform more smoothly if I used a smaller sample rate. The job of the line-following controller is to generate an angular velocity command to the robot. This angular velocity command gets mapped into wheel velocity commands, which is then sent to the wheel velocity controller. Thus intuitively, it makes sense that the wheel velocity control loop should have higher bandwidth than the line-following controller. When the line-following controller requests an angular velocity, and thus wheel velocities, the wheel velocity controller should achieve the desired wheel velocities much faster than new angular velocity requests are coming in. After some trial and error, setting the line-following sample rate at 40 Hz, which is 5 times slower than the wheel velocity control loop sample rate, seems to work well.

### Angle measurement

The Sunfounder 8 channel IR sensor array outputs an analog integer value for each sensor between 0 and 255. If the sensor is over a white line, the output value goes “high” around 245-250. If the sensor is over the black electrical tape line, the output value is goes “low” anywhere from 0 to 200 depending on how much the sensor is on top of the line. One strategy for calculating the robot heading angle is to first sum the weighted angles of all of the sensors, which we will call the effective angle \theta_{eff}*θ**e**f**f*. This method of interpreting an effective angle from the IR sensor outputs is based on the Arduino library for the Pololu QTR Reflectance Sensors (specifically the`readLine`method in the QTRSensors.cpp file):

- Wiki: <https://www.pololu.com/docs/0J19/all>
- GitHub: <https://github.com/pololu/qtr-sensors-arduino/blob/master/QTRSensors.cpp>

If the sensor output is “stronger”, i.e, more overlapped with the black line, then that sensor angle will be be weighted more heavily in the effective angle. The effective angle is calculated by:
\theta_{eff} = \frac{0\times(255-val_0) + 1\times(255-val_1) +...+7\times(255-val_7)}{(255-val_0)+(255-val_1)+...+(255-val_7)} \\ =\frac{\sum^{7}_{i=0}i\times(255-val_i)}{\sum^{7}_{i=0}(255-val_i)}*θ**e**f**f*​=(255−*v**a**l*0​)+(255−*v**a**l*1​)+...+(255−*v**a**l*7​)0×(255−*v**a**l*0​)+1×(255−*v**a**l*1​)+...+7×(255−*v**a**l*7​)​=∑*i*=07​(255−*v**a**l**i*​)∑*i*=07​*i*×(255−*v**a**l**i*​)​
If sensor 0 on the left is completely over the black line and all other sensors are detecting white, then \theta_{eff} = 0*θ**e**f**f*​=0. If sensor 7 is completely over the black line and all other sensors are detecting white, then \theta_{eff}=7*θ**e**f**f*​=7. Because there are an even number of sensors, if the sensor array is centered on the line, then \theta_{eff}=7*θ**e**f**f*​=7. For a sensor array with N*N* sensors, \theta_{eff}=\frac{N-1}{2}*θ**e**f**f*​=2*N*−1​ when the sensor array is centered on the line.

Due to the sensor noise and environmental factors, the sensor output fluctuates around 245-250 when the sensor is above a white surface and we do not want these values to contribute to the \theta_{eff}*θ**e**f**f* calculation. Thus, I only count val_i*v**a**l**i* in the calculation of \theta_{eff}*θ**e**f**f* when the val_i&gt;val_{min}*v**a**l**i*>*v**a**l**m**i**n*, where val_{min}*v**a**l**m**i**n* represents the minimum value of the sensor output when there is a partial detection of the black line. val_{min}*v**a**l**m**i**n* was obtained experimentally as 210.

If none of the sensors detect line during a given sampling instant, then we do not want to calculate \theta_{eff}*θ**e**f**f* because we would be dividing by 0 in the numerator. Furthermore, it is possible and likely that the sensor array drifted completely off the black line and needs to turn quickly to recover. If \theta_{eff,avg,prev} &lt; 2*θ**e**f**f*,*a**v**g*,*p**r**e**v*<2, then the robot probably drifted off to the right side of the black line so set \theta_{eff}=\theta_{eff,min}=0*θ**e**f**f*=*θ**e**f**f*,*m**i**n*=0. \theta_{eff,avg}*θ**e**f**f*,*a**v**g* is a filtered version of \theta_{eff}*θ**e**f**f* and will be explained in the next section. The reason I check \theta_{eff,avg,prev} &lt; 2*θ**e**f**f*,*a**v**g*,*p**r**e**v*<2 rather than \theta_{eff,avg,prev} &lt; 0*θ**e**f**f*,*a**v**g*,*p**r**e**v*<0 is that we don’t want to overestimate \theta_{eff,avg}*θ**e**f**f*,*a**v**g* if the sensor array was close to being centered on the black line in the previous sampling period. This value should be determined experimentally. Similarly if \theta_{eff,avg,prev} &gt; 5*θ**e**f**f*,*a**v**g*,*p**r**e**v*>5, then the robot probably drifted off to the left side of the black line so set \theta_{eff}=\theta_{eff,max}=7*θ**e**f**f*=*θ**e**f**f*,*m**a**x*=7.

### Filtering the angle

Instead of using \theta_{eff}*θ**e**f**f* directly as the sensor measurement, I smoothed it out using a low pass filter:
\theta_{eff,avg} = \alpha(\theta_{eff,avg,prev})+(1-\alpha)\theta_{eff}*θ**e**f**f*,*a**v**g*​=*α*(*θ**e**f**f*,*a**v**g*,*p**r**e**v*​)+(1−*α*)*θ**e**f**f*​
I chose \alpha = 0.3*α*=0.3 by trial and error. TODO: Calculate the cut-off frequency of this low-pass filter based on the sample rate being used.

### Defining the angle error

Based on the coordinate system of the robot body, a positive angular velocity corresponds to a left turn and a negative angular velocity corresponds to a right turn. If the output of the controller is the desired angular velocity, then we have:
u (\omega_{des}) = K_p\theta_{error} + K_d\dot\theta_{error}*u*(*ω**d**e**s*​)=*K**p*​*θ**e**r**r**o**r*​+*K**d*​*θ*˙*e**r**r**o**r*​
If \theta_{eff,avg} &lt; 3.5*θ**e**f**f*,*a**v**g*​<3.5 which means the sensor array center is pointing to the right of the black line, then the robot should turn left: \omega_{des} &gt; 0*ω**d**e**s*​>0. Thus, the angle error should be defined as \theta_{error}=3.5-\theta_{eff,avg}*θ**e**r**r**o**r*​=3.5−*θ**e**f**f*,*a**v**g*​. The derivative of the angle error is approximated as:
\dot\theta_{error}=\frac{\theta_{error}-\theta_{error,prev}}{T_{steer}}*θ*˙*e**r**r**o**r*​=*T**s**t**e**e**r*​*θ**e**r**r**o**r*​−*θ**e**r**r**o**r*,*p**r**e**v*​​
where T_{steer}*T**s**t**e**e**r*​ is the sampling period of the line-following controller.

### Clamping wheel speeds based on physical limits

This is directly from the Control of Mobile Robots course on Coursera. This strategy works extremely well because it will always reduce the linear velocity command in favor of achieving the desired angular velocity when the required wheel speeds exceed the maximum wheel speed. This means that when the robot is following a straight line, it will probably be moving at the commanded linear velocity. However, when following a high curvature path, the linear velocity command may be reduced in order to achieve the desired angular velocity required to follow the path.

The first step is to experimentally determine the maximum linear wheel velocity, v_{w,max}*v**w*,*m**a**x*. For simplicity, assume that both motors have the same maximum velocity. If not, use the lower of the two values as the maximum velocity. The units of v_{w,max}*v**w*,*m**a**x* are in meters per second to be consistent with the MIT 2.12 Lab Exercises. Next calculate the robot’s maximum linear velocity, v_{max}*v**m**a**x*. The maximum linear velocity occurs when the angular velocity is zero. Hence, we have:
v_{max} = (v_{l,max} + v_{r,max})/2 = v_{w,max}*v**m**a**x*​=(*v**l*,*m**a**x*​+*v**r*,*m**a**x*​)/2=*v**w*,*m**a**x*​
Also calculate the robot’s maximum angular velocity, \omega_{max}*ω**m**a**x*​. The maximum angular velocity occurs when the left and right wheels are spinning in equal and opposite directions. This gives:
\omega_{max} = \frac{1}{L}(v_{r,max} - v_{l,max}) = \frac{1}{L}(2v_{w,max})=\frac{v_{w,max}}{b}*ω**m**a**x*​=*L*1​(*v**r*,*m**a**x*​−*v**l*,*m**a**x*​)=*L*1​(2*v**w*,*m**a**x*​)=*b**v**w*,*m**a**x*​​
where L=2b*L*=2*b* is the track width.

The next step is to clamp the commanded velocity and angular velocity if they go outside of the range of the minimum or maximum values. This ensures that -v_{max} \leq v_{des} \leq v_{max}−*v**m**a**x*≤*v**d**e**s*≤*v**m**a**x* and -\omega_{max} \leq \omega_{des} \leq \omega_{max}−*ω**m**a**x*≤*ω**d**e**s*≤*ω**m**a**x*.

Next compute the desired linear wheel velocities needed to ensure the new desired angular velocity:
v_{r,des}=(v_{des} + \omega_{des}b) \\ v_{l,des}=(v_{des} - \omega_{des}b)*v**r*,*d**e**s*​=(*v**d**e**s*​+*ω**d**e**s*​*b*)*v**l*,*d**e**s*​=(*v**d**e**s*​−*ω**d**e**s*​*b*)

Now find the maximum and minimum desired linear wheel speeds:
v_{rl,max} = \max(v_{r,des}, v_{l,des}) \\ v_{rl,min} = \min(v_{r,des}, v_{l,des})*v**r**l*,*m**a**x*​=max(*v**r*,*d**e**s*​,*v**l*,*d**e**s*​)*v**r**l*,*m**i**n*​=min(*v**r*,*d**e**s*​,*v**l*,*d**e**s*​)

If the desired wheel velocities exceed the maximum or minimum, reduce both desired wheel velocities by the (same) exceeded amount:

```arduino
if (wv_RL_max > wv_max) {
  desiredWV_R = desiredWV_R - (wv_RL_max - wv_max);
  desiredWV_L = desiredWV_L - (wv_RL_max - wv_max);
} else if (wv_RL_min < -wv_max) {
  desiredWV_R = desiredWV_R - (wv_RL_min + wv_max);
  desiredWV_L = desiredWV_L - (wv_RL_min + wv_max);
}
```

Reducing both desired wheel velocities by the same amount ensures that only the effective commanded linear velocity will be decreased if both the desired linear velocity and angular velocity cannot be achieved simultaneously.

## References

### Projects

1. <https://www.allaboutcircuits.com/projects/how-to-build-a-robot-line-follower/>
2. <https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010>
3. <http://people.csail.mit.edu/peterkty/teaching.htm>
4. <https://github.com/mit212>
5. <https://github.com/vanadiumlabs/arbotix_ros/blob/indigo-devel/arbotix_python/src/arbotix_python/diff_controller.py>
6. <https://github.com/arduinoNube/digital-pid-classroom-demo/blob/master/main.c>
7. <https://github.com/PINTO0309/zumo32u4> - zumo32u4 robot with RPLidar running on ROS
8. <https://www.youtube.com/watch?v=OJNNm6iMOKk> - # Using a Lidar for Robot Navigation in a Room - Michael E Anderson, The PTR Group, Inc.

### Arduino

1. <https://hackaday.com/2015/07/28/embed-with-elliot-there-is-no-arduino-language/>
2. <https://learn.adafruit.com/multi-tasking-the-arduino-part-1/using-millis-for-timing>

### Raspberry Pi

1. <https://www.raspberrypi-spy.co.uk/2017/04/manually-setting-up-pi-wifi-using-wpa_supplicant-conf/>
2. 

### Interrupts

1. <https://forum.arduino.cc/index.php?topic=134931.0>
2. <http://gammon.com.au/interrupts>

### TimerOne library

1. <https://www.brainy-bits.com/speed-sensor-with-arduino/>
2. Examples - <https://github.com/PaulStoffregen/TimerOne/tree/master/examples>
3. Official Arduino site documentation - <http://playground.arduino.cc/Code/Timer1>

### Motor Control

1. <http://blog.solutions-cubed.com/pid-motor-control-with-an-arduino/>
2. <https://github.com/merose/ROSRobotControl/blob/master/ROSRobotControl.ino>
3. <https://answers.ros.org/question/275080/arduino-ros-communication-using-dc-motor-with-encoder/>
4. <https://github.com/PacktPublishing/ROS-Programming-Building-Powerful-Robots/blob/cec8997d36ccc5eac15a7c0627cac96c3c7a0218/Module01/Chapter08/chapter8_tutorials/src/c8_robot_with_motors_and_encoders/c8_robot_with_motors_and_encoders.ino>

### ROS

1. <http://wiki.ros.org/roscpp/Overview/Timers>
2. <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>
3. <http://wiki.ros.org/catkin/Tutorials/using_a_workspace>
4. <http://wiki.ros.org/ROS/Tutorials/CreatingPackage>
5. <http://wiki.ros.org/catkin/commands/catkin_make>
6. <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>
7. <https://answers.ros.org/question/199363/error-run-teleop_twist_keyboard-package/>
8. <http://wiki.ros.org/teleop_twist_keyboard>
9. <http://wiki.ros.org/differential_drive>
10. <http://wiki.ros.org/ros_arduino_bridge>
11. <https://answers.ros.org/question/245514/dc-motor-encoder-python/>
12. <https://answers.ros.org/question/223467/how-to-connect-wheel-encoder-to-ros-computer/>
13. <http://wiki.ros.org/arbotix_python/diff_controller>
14. <https://answers.ros.org/question/12684/using-multiple-arduinos-running-multiple-nodes/>







<iframe src="https://monetizejs.com/authorize?immediate=true&amp;response_type=token&amp;client_id=ESTHdCYOi18iLhhO&amp;redirect_uri=https%3A%2F%2Fstackedit.io%2Fapp%23&amp;q=1554902422702" style="box-sizing: border-box; color: rgba(0, 0, 0, 0.75); font-family: Lato, &quot;Helvetica Neue&quot;, Helvetica, sans-serif; font-size: 16px; font-style: normal; font-variant-ligatures: common-ligatures; font-variant-caps: normal; font-weight: 400; letter-spacing: normal; orphans: 2; text-align: start; text-indent: 0px; text-transform: none; white-space: normal; widows: 2; word-spacing: 0px; -webkit-text-stroke-width: 0px; text-decoration-style: initial; text-decoration-color: initial; width: 1px; height: 1px; position: absolute; top: -100px;"></iframe>