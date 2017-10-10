[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

## Project 2: Amazon Pick and Place Challenge
###### Udacity Robotics Nanodegree
###### September 2017


[//]: # (Image References)

[image1]: ./Kuka\KR210/req-challenge.gif
[image2]: ./path/Kuka_KR210_Orientation.png
[image3]: ./Kuka\KR210/gazebo-demo.gif
[image4]: ./Kuka\KR210/moveit-demo.gif
[image5]: ./Kuka\KR210/rviz-demo.gif
[image6]: ./Joint\Types/joint-types-and-degrees-of-freedom-01.png
[image7]: ./Serial\Manipulator\Types/Anthropomorphic_Manipulator(RRR).png

###
###
###

### Overview

###### The goal of this project is to program a Serial Manipulator to pick up objects and place them in a bin. The project is based on Amazon's Pick and Place Challenge.

![alt text][image1]

##### Specifically, the objectives of this project are to:

1. Identify the location of a single object on a 9-position shelf(3x3 configuration)
2. Maneuver the Kuka KR210 - a 6 degree of freedom Serial Manipulator - toward the object
3. Grasp the object
4. Chart a path to the destination bin
5. Maneuver the Kuka KR210 along the charted path
6. Sucessfully place the object in the destination bin

###

If you'd like to watch the Kuka KR210 in action, click [here.](https://youtu.be/rV0lWmJ7uOM)

Check out the Amazon Robotics Challenge [here.](https://youtu.be/yVIRLao1E28)

###

###

*We operate the 6 D.o.F. arm through ROS Kinetic (ran through Linux Ubuntu 16.0) and commands are written in Python.*

The code driving this project and interacting with ROS can be found at `IK_server.py`


### Environment Setup

`Robotic VM V2.0.1` is the directory that contains the Linux Boot image:`Ubuntu 64-bit Robo V2.0.1.ova`

If prompted, the Linux system password is `robo-nd`

Source and build the project with:
```
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws/
catkin_make

```

To toggle  `demo mode`  on and off, set the `flag=` to true or false with:
```
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/launch/
nano inverse_kinematics.launch

```

Now, we're reading to start out project up!

```
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/
./safe_spawner.sh
```
Give the environments some time to get started up (sometimes up to 30s)

Now, open a new terminal and type:
```
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/
rosrun kuka_arm IK_server.py
```

Now, navigate to the RViz window and click `continue` to begin the KR210 arm.

### Notebook Analysis

*README in PROGRESS*

