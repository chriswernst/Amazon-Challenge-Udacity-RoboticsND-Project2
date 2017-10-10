[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

## Project 2: Amazon Pick and Place Challenge
##### Udacity Robotics Nanodegree
###### September 2017

###
###

### Overview

###### The goal of this project is to program a Serial Manipulator to pick up objects and place them in a bin. The project is based on Amazon's Pick and Place Challenge.
###
###

![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Kuka%20KR210/req-challenge.gif?raw=true)

##### Specifically, the objectives of this project are to:

1. Identify the location of a single object on a 9-position shelf(3x3 configuration)
2. Maneuver the Kuka KR210 - a 6 degree of freedom Serial Manipulator - toward the object
3. Grasp the object
4. Chart a path to the destination bin
5. Maneuver the Kuka KR210 along the charted path
6. Sucessfully place the object in the destination bin

###

If you'd like to watch the Kuka KR210 in action, click [**here.**](https://youtu.be/rV0lWmJ7uOM)

Check out the Amazon Robotics Challenge [**here.**](https://youtu.be/yVIRLao1E28)

###

We operate the 6 D.o.F. arm through **ROS Kinetic** (ran through Linux Ubuntu 16.0) and commands are written in **Python**.

The code driving this project and interacting with ROS can be found at `IK_server.py`


### Environment Setup

`Robotic VM V2.0.1` is the directory that contains the Linux Boot image:`Ubuntu 64-bit Robo V2.0.1.ova`

If prompted, the Linux system password is `robo-nd`

To run projects from this repository you need version **7.7.0+** of Gazebo.

Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```

If your gazebo version is not **7.7.0+**, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

If you do not have an active ROS workspace, you can create one with:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download the project repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window, to update the packages and set permissions for our executables:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```

Source and build the project with:
```sh
$ source ~/catkin_ws/devel/setup.bash
$ cd ~/catkin_ws/
$ catkin_make
```

Add following to the bottom of your `.bashrc` file -- which is typically found in the home directory `/home/robond/`
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

Although optional, I found it very useful to also add a line to set the directory to where our scripts live. So that every time a new terminal window is opened, we don't have to set the directory. If desired, also add to `.bashrc`:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/
```
To toggle  `demo mode` off, set `<param name="demo" value=false` in the `inverse_kinematics.launch` file with:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/launch/
$ nano inverse_kinematics.launch
```
Once we have our `IK_server.py` code ready, we're going to keep the `value=false`. If you do not have `IK_server.py` code ready, keep `value=true`.

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the `spawn_location` argument in `target_description.launch` file under 
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/launch/
$ nano target_description.launch
```
0-9 are valid values for spawn_location with 0 being random mode.

Now, we're finally ready to launch our project!
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/
$ ./safe_spawner.sh
```
Give the environments some time to get started up (sometimes up to 30s - especially if you're running a VM).

Once **Gazebo** and **RViz** are up and running, make sure you see following in the Gazebo world:
	- KR210 Robot
	- Shelf
	- Blue cylindrical target on one of the shelves
	- Dropbox next to the robot
	
Now, open a new terminal and type:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/
$ rosrun kuka_arm IK_server.py
```
If you're getting errors that are making the robot act unusual, press `ctrl+c` in the terminal windows and launch `./safe_spawner.sh` and `rosrun kuka_arm IK_server.py` again.

Now, navigate to the **RViz** window and click `continue` to begin the motion of the KR210 arm.

### Code Analysis

`IK_server.py` houses our code that will calculate the joint variables for the Kuka KR210.  The KR210 has 6 *Revolute Type* joints. Counting up from the base link, we have: `theta1` `theta2` `theta3` `theta4` `theta5` `theta6`
<p align="center">
    ![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Diagrams/l20-inverse-kinematics-02.png?raw=true)
</p>
The first 3 joints `theta1` `theta2` `theta3` have the largest impact on the location of the end effector, or gripper; while the last 3 joints `theta4` `theta5` `theta6` make up the *spherical wrist*, with `theta5` being the Wrist Center or `WC`.


For reference, a real Kuka KR210 looks like this:
<p align="center">
    <img src="https://media.robots.com/images/1363894998_1.jpg">
</p>

A quick refresher on joint types:

![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Joint%20Types/joint-types-and-degrees-of-freedom-01.png?raw=true)

We will primarily be solving the problem of Inverse Kinematics -- which is taking a point in the Cartesian Space (the location of the end effector), and determining which combinations of `theta1-6` are required to reach that point. This is no easy task, and often has more than one solution, or even no solutions!

![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Diagrams/forward-kinematics-01.png?raw=true)




*README in PROGRESS*

