## Exercise 3.12

a) Inside the package that you created in the exercise above, create a new file named **square_move.py**.

b) Inside this file, write the code to move the robot. The robot must perform a square movement. When it finishes, it has to print the covariance of the filter particles into the screen.

c) Test that your code works (the robot moves in a square).

d) When you've tested that it works, add the necessary code so that your program does the following:

1. First of all, it calls the necessary service in order to disperse the particles all around the environment.
2. When the particles are randomly dispersed, it performs the square movement.
3. When it has finished doing the movement, it checks the particles' covariance.
4. If this covariance is less than 0.65, this means that the robot has localized itself correctly. Then, the program will end. If this covariance is greater than 0.65, it will repeat the whole the process (disperse the particles, perform the movement, check the covariance...).

### *Provided Solution

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty, EmptyRequest
import time
import math

class MoveHusky():
    
    def __init__(self):
        
        # Init Publisher
        self.husky_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        # Init Subscriber
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.sub_callback)
        self.sub_msg = PoseWithCovarianceStamped()
        # Initialize Service Client
        rospy.wait_for_service('/global_localization')
        self.disperse_particles_service = rospy.ServiceProxy('/global_localization', Empty)
        self.srv_request = EmptyRequest()
        # Other stuff
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)
        
    def shutdownhook(self):
        
        # works better than the rospy.is_shut_down()
        self.stop_husky()
        self.ctrl_c = True

    def stop_husky(self):
        
        rospy.loginfo("Shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        i = 0
        
        while i < 20:
            self.husky_vel_publisher.publish(self.cmd)
            self.rate.sleep()
            i += 1

    def move_forward(self, linear_speed=0.5, angular_speed=0.0):
        
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0
        
        while i < 50:
            self.husky_vel_publisher.publish(self.cmd)
            self.rate.sleep()
            i += 1
        
    def turn(self, linear_speed=0.0, angular_speed=0.8):
        
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0
        
        while i < 25:
            self.husky_vel_publisher.publish(self.cmd)
            self.rate.sleep()
            i += 1
    
    def move_square(self):
        
        i = 0
        
        while not self.ctrl_c and i < 4:
            # Move Forwards
            rospy.loginfo("######## Going Forwards...")
            self.move_forward()
            self.stop_husky()
            # Turn
            rospy.loginfo("######## Turning...")
            self.turn()
            self.stop_husky()
            i += 1
            
        self.stop_husky()
        rospy.loginfo("######## Finished Moving in a Square")
        
    def call_service(self):
        
        rospy.loginfo("######## Calling Service...")
        result = self.disperse_particles_service(self.srv_request)
        
    def sub_callback(self, msg):
        
        self.sub_msg = msg

    def calculate_covariance(self):
        
        rospy.loginfo("######## Calculating Covariance...")
        cov_x = self.sub_msg.pose.covariance[0]
        cov_y = self.sub_msg.pose.covariance[7]
        cov_z = self.sub_msg.pose.covariance[35]
        rospy.loginfo("## Cov X: " + str(cov_x) + " ## Cov Y: " + str(cov_y) + " ## Cov Z: " + str(cov_z))
        cov = (cov_x+cov_y+cov_z)/3
        
        return cov
        
            
if __name__ == '__main__':
    rospy.init_node('move_husky_node', anonymous=True)
    MoveHusky_object = MoveHusky()
    
    cov = 1
    
    while cov > 0.65:
        MoveHusky_object.call_service()
        MoveHusky_object.move_square()
        cov = MoveHusky_object.calculate_covariance()
        rospy.loginfo("######## Total Covariance: " + str(cov))
        if cov > 0.65:
            rospy.loginfo("######## Total Covariance is greater than 0.65. Repeating the process...")
        else:
            rospy.loginfo("######## Total Covariance is lower than 0.65. Robot correctly localized!")
            rospy.loginfo("######## Exiting...")
```



### Data for Exercise 3.12

Check the following notes in order to complete the exercise: 

Note 1: Keep in mind that in order to be able to call this service, you need to have the amcl node running.
Note 2: The particles covariance is published into the /amcl_pose topic.
Note 3: The particles covariance is published into a matrix. This matrix looks like this:

![img](assets/ex3p12_particle_covariance.png)

The only values that you need to pay attention to is the first one (which is the covariance in x), the 8th one (which is the covariance in y), and the last one (which is the covariance in z).

> The open loop controller running in square_move.py will not work correctly if teleop mode is running in another terminal. The robot will be moving extremely slowly. Specifically, I had `roslaunch husky_navigation_launch keyboard_teleop.launch` running in another terminal. Make sure this is not currently running in another terminal!

# Chapter 4: Path Planning Part 1

## Summary

Estimated time to completion: **3 hours**

What will you learn with this unit?

- Visualize Path Planning in Rviz
- Basic concepts of the move_base node
- What is the Global Planner?
- What is the Global Costmap?

We're arriving at the end of the course, guys! For now, we've seen how to create a map of an environment, and how to localize the robot in it. So, at this point (and assuming everything went well), we have all that we need in order to perform Navigation. That is, we're now ready to plan trajectories in order to move the robot from pose A to pose B.

In this chapter, you'll learn how the Path Planning process works in ROS, and all of the elements that take place in it. But first, as we've been doing in previous chapters, let's have a look at our digital best friend, RViz.

### Visualize Path Planning in Rviz

As you've already seen in previous chapters, you can also launch RViz and add displays in order to watch the Path Planning process of the robot. For this chapter, you'll basically need to use 3 elements of RViz:

- Map Display (Costmaps)
- Path Displays (Plans)
- 2D Tools

## Exercise 4.1

a) Execute the next command in order to launch the move_base node.

Execute in WebShell #1:

```
roslaunch husky_navigation move_base_demo.launch
```

b) Open the Graphic Interface and execute the following command in order to start RViz.

Execute in WebShell #2:

```
rosrun rviz rviz
```

c) Properly congifure RViz in order to visualize the necessary parts.

### Visualize Costmaps

- Click the Add button under Displays and chose the Map element.
- Set the topic to **/move_base/global_costmap/costmap**in order to visualize the global costmap
- Change the topic to **/move_base/local_costmap/costmap**in order to visualize the local costmap.
- You can have 2 Map displays, one for each costmap.

> You can add more than one Map display at a time in Rviz to visualize the global costmap and local costmap at the same time.

> The local_costmap displays the free and occupied space around the robot based on the current laser sensor readings.

### Visualize Plans

- Click the Add button under Displays and chose the Path element.
- Set the topic to **/move_base/NavfnROS/plan** in order to visualize the global plan.
- Change the topic to **/move_base/DWAPlannerROS/local_plan** in order to visualize the local plan.
- You can also have 2 Path displays, one for each plan.

> There is another possible topic to subscribe to called /move_base/DWAPlannerROS/global_plan. This global_plan path is always displayed in Rviz while the trajectory is being executed whereas the topic /move_base/NavfnROS/plan is only displayed initially after a new "2D Nav Goal" is commanded and then it disappears. This happens because /move_base/NavfnROS/plan is only published once after a new goal is commanded, whereas both /move_base/DWAPlannerROS/local_plan and /move_base/DWAPlannerROS/local_plan are published at a rate of around 5 Hz while the robot is trying to reach the goal. You can check the publishing frequency this by running the command: `rostopic hz /move_base/DWAPlannerROS/local_plan`.

d) Use the 2D Pose Estimate tool in order to provide an initial pose for the robot. 

![img](assets/ch4_2d_pose_estimate_rviz.png)

e) Use the 2D Nav Goal tool in order to send a goal pose to the robot. Make sure to select an unoccupied (dark grey) or unexpected (light grey) location. 

![img](assets/ch4_2d_nav_goal_rviz.png)

### Data for Exercise 4.1

Check the following notes in order to complete the exercise:

**Note 1**: Bear in mind that if you don't set a 2D Nav Goal, the planning process won't start. This means that until you do, you won't be able to visualize any plan in RViz.

**Note 2**: In order for the 2D tools to work, the Fixed Frame at Rviz must be set to map.

### Expected Result for Exercise 4.1

Global Costmap:

![img](assets/ex4p1_rviz_global_costmap.png)

Local Costmap:

![img](assets/ex4p1_rviz_local_costmap.png)

Global Plan:

![img](assets/ex4p1_rviz_planning1_clean.png)

Local Plan (in red) and Global Plan (in green):

![img](assets/ex4p1_global_vs_local_clean.png)

**REMEMBER**: Remember to save your RViz configuration in order to be able to load it again whenever you want. If you don't remember how to do it, check the Mapping Chapter.

That's awesome, right? But what has just happened? What was that 2D Nav Goal tool I used in order to move the robot? And what's a Costmap? How does ROS calculate the trajectories?

Keep calm!! By the end of this chapter, you'll be able to answer all of those questions. But let's go step by step, so that you can completely understand how the whole process works.

### The move_base package

The move_base package contains the **move_base node**. Doesn't that sound familiar? Well, it should, since you were introduced to it in the Basic Concepts chapter! The move_base node is one of the major elements in the ROS Navigation Stack, since it links all of the elements that take place in the Navigation process. Let's say it's like the Architect in Matrix, or the Force in Star Wars. Without this node, the ROS Navigation Stack wouldn't make any sense!

Ok! We understand that the move_base node is very important, but... what is it exactly? What does it do? Great question!

The **main function of the move_base node is to move the robot from its current position to a goal position**. Basically, this node is an implementation of a *SimpleActionServer*, which takes a goal pose with message type *geometry_msgs/PoseStamped*. Therefore, we can send position goals to this node by using a *SimpleActionClient*.

This Action Server provides the topic **move_base/goal**, which is the input of the Navigation Stack. This topic is then used to provide the goal pose.

## Exercise 4.2

a) In a WebShell, visualize the *move_base/goal* topic.

b) As you did in the previous exercise, send a goal to the robot by using the 2D Nav Goal tool in RViz.

c) Check what happens in the topic that you are listening to.

### Expected Result for Exercise 4.2

![img](assets/ex4p2_send_goal_msg.png)

---

So, each time you set a Pose Goal using the 2D Nav Goal tool from RViz, what is really happening is that a new message is being published into the move_base/goal topic.

Anyway, this is not the only topic that the move_base Action Server provides. As every action server, it provides the following 5 topics:

- **move_base/goal (move_base_msgs/MoveBaseActionGoal)**
- **move_base/cancel (actionlib_msgs/GoalID)**
- **move_base/feedback (move_base_msgs/MoveBaseActionFeedback)**
- **move_base/status (actionlib_msgs/GoalStatusArray)**
- **move_base/result (move_base_msgs/MoveBaseActionResult)**

## Exercise 4.3

Without using Rviz, send a pose goal to the move_base node.

a) Use the command line tool in order to send this goal to the Action Server of the move_base node.

b) Visualize through the webshells all of the topics involved in the action, and check their output while the action is taking place, and when it's done.