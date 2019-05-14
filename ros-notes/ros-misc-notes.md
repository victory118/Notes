## Publishing Transforms

### static_transform_publisher

The **static_transform_publisher** is a node that can be run using the command line as:

```
static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
```

The command can also be launched using a launch file:

```xml
<launch>
    <node pkg="tf" type="static_transform_publisher" name="name_of_node" 
          args="x y z yaw pitch roll frame_id child_frame_id period_in_ms">
    </node>
</launch>
```

### Loading parameters from YAML file through the launch file

If you want to define parameters of a node in a YAML file and then load it through the launch file, add the following line inside the **<node>** tag:

```xml
<rosparam file="$(find my_mapping_launcher)/params/gmapping_params.yaml" command="load" />
```

This will have the exact same result as if the parameters are loaded directly through the launch file.

### Increase rosserial buffer size

<https://answers.ros.org/question/73627/how-to-increase-rosserial-buffer-size/>

## Services

### List all ROS services available

```
rosservice list
```

### Calling a service

```
rosservice call /service_name "argument"
```

Example:

```
rosservice call /static_map "{}"
```

This calls the **static_map** service with no arguments.



## Running ROS on Ubuntu laptop and Raspberry Pi

### 1. Both Ubuntu laptop and RPi are connected to external Wifi access point

In this scenario, assume the Ubuntu laptop is connected to an external Wifi access point like a home or school router and the RPi is connected to the same Wifi access point. Note that the RPi can also serve as a Wifi access point, but we will get to that scenario later.

On the Ubuntu laptop, open a terminal and check the IP(network) address of the host by running:

```
$ hostname -I
192.168.1.7
```
Based on my understanding, the `-I` (uppercase) option should almost always be used over the `-i`(lowercase) option. The output from the Ubuntu laptop terminal was 192.168.1.7.

Next export this IP address so the other computer running ROS (RPi) can identify it. Open a terminal on the Ubuntu laptop and run:
```
$ export ROS_IP=192.168.1.7
```
You can check that this value is set correctly by running `echo $ROS_IP`.

Next go to the RPi and set the IP address of the remote computer (Ubuntu laptop) that will be running the ROS master node (i.e roscore). Open a terminal in the Rpi and run:
```
$ export ROS_MASTER_URI=http://192.168.1.7:11311/
```

Next export the IP address of the RPi so it can be identified by the ROS server. As before, you can check the IP address of the RPi by running `hostname -I`. The output was 192.168.1.11 but the output could be different every time. Then export the IP address in the RPi by running:
```
$ export ROS_IP=192.168.1.11
```
Test the communication by using the turtlesim node. On the Ubuntu laptop, start the ROS master node by running `roscore`.

On the RPi, open a terminal and start the turtlesim node:
```
$ rosrun turtlesim turtlesim_node
```

Now we will control the turtle on the Ubuntu laptop using the teleop node. Open a new terminal on the Ubuntu laptop and run:
```
$ rosrun turtlesim turtle_teleop_key
```
Note that when you open a new terminal to run a new ROS node, you have to export the IP address again. To avoid doing this every time, you can add the export command in the .bashrc file so it runs every time you open a new terminal. The same can be done on the RPi. You should now be able control the turtlesim node running in the RPi by pressing the keyboard keys on the Ubuntu laptop.

The video tutorial I followed can be found here: https://www.youtube.com/watch?v=S_9yi-u_Aeg

## Setting up SSH connection from Ubuntu laptop to Ubuntu MATE on RPi

Several settings have to be configured on Ubuntu MATE on the RPi in order to SSH from the Ubuntu laptop.

If the SSH server is not installed, do so by running:
```
$ sudo apt update
$ sudo apt-get install openssh-server
```
Check the status of the SSH service to see if it is enabled by running:
```
$ sudo service ssh status
```
If the service is disabled, then enable it by running:
```
$ sudo systemctl enable ssh
```
Then restart the SSH server by running:
```
$ sudo service ssh restart
```
It's possible that you have to change the SSH configuration file to allow password login. To do this you need to edit the **sshd_config** file by running:
```
$ sudo nano /etc/ssh/sshd_config
```
The `sudo` prefix gives you write permission for the file. Navigate down to the line where it says `PermitRootLogin`. The default setting is `PermitRootLogin prohibit-password`, which means that SSH login using a password is prohibited. Change this setting so it says `PermitRootLogin yes` and save the changes.

If you are not sure which password to use for the SSH login, then change it. To change the root password on Ubuntu MATE, run the command:
```
$ sudo passwd root
```
Then follow the instructions to change the password.

After following the above steps, SSH login from the Ubuntu laptop to the RPi by running:
```
$ ssh victor@192.168.1.11
```
Here, `victor` is the username on the RPi where SSH was installed and the IP address can be found by running `hostname -I`. Enter in the root password that was configured in the previous step.

If you encounter an error message saying something like "Add correct host key in known_hosts," then you may try running:
```
$ ssh-keygen -R *ip_address_or_hostname*
```
where `ip_address_or_hostname` is the IP address of the RPi. This will remove the problematic IP or hostname from known_hosts file and try to connect again. Reference: https://serverfault.com/questions/321167/add-correct-host-key-in-known-hosts-multiple-ssh-host-keys-per-hostname

### Serial Communication with Arduino

Upload the .ino Arduino code into the Arduino board.

Open a new terminal and start `roscore`.

Open another terminal and run the **rosserial client** application:

```
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

Running this enables communication between the ROS serial node running on the laptop and the Arduino. If necessary, replace the name of the USB port `/dev/ttyACM0` with the correct one to which the Arduino is connected.

> ### Teleop mode

Running the `teleop_twist_keyboard` node will allow you to publish velocity commands to the `/cmd_vel` topic using the keyboard to send commands:

```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Pressing the appropriate keys will send velocity and angular velocity commands to the robot, which should cause it to move accordingly. To see the commanded velocity being published to the robot, run the command:

```
$ rostopic echo /cmd_vel
```

## Problems encountered and possible solutions

### There is no output on the terminal when I launch a node use a launch file.
Within the `<node>` tag, make sure that you have the **output** argument set to **"screen"**. For example, the contents of the launch file below launches the **call_map_service** node from the **get_map_data** package from the **call_map_service.py** executable file:
```xml
<launch>
    <node pkg="get_map_data"
    type="call_map_service.py"
    name="call_map_service"
    output="screen">
    </node>
</launch>
```

### rospy.loginfo does not print anything to the terminal

Make sure that the node is initialized before using rospy.loginfo. For example, see the code below:

```python
topic = 'chatter'
    pub = rospy.Publisher(topic, String)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo("I will publish to the topic %s", topic)
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        rospy.sleep(0.1)
```

Reference: <http://wiki.ros.org/rospy_tutorials/Tutorials/Logging>

### How to call rosservice in terminal with Empty service data type

If you want to call a service from the terminal and the service uses the Empty service (srv) data type, then you can call it by running:

``` 
$ rosservice call /name_of_service arg1 arg2
```

Here, you should omit `arg1` and `arg2` (i.e., do not input any arguments) and submit the command after entering `/name_of_service`.

### Module not found error

It's possible that the ROS catkin workspace is not using the correct Python path. You can check this by running `echo $PYTHONPATH`. In my case, one of the Python paths contained a link to the Python3 version. ROS Kinetic does not work with Python3, only Python2.7. The reason my ROS workspace was set up incorrectly is that I initialized a catkin workspace in the terminal while it was in the Anaconda Python environment, which uses Python3.

In the future when doing anything with ROS or catkin, first ensure the Anaconda environment is deactivated in the terminal by running: `conda deactivate`. Then remove the catkin workspace and initialize a new catkin workspace. At the moment, I think it is easier to just create a new workspace than it is to fix the old one. Now when you reinstall the missing modules and packages, your system should have no problems finding them.

### Multiple catkin workspaces

