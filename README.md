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

**1.** Identify the location of a single object on a 9-position shelf(3x3 configuration)
###
**2.** Maneuver the Kuka KR210 - a 6 degree of freedom Serial Manipulator - toward the object
###
**3.** Grasp the object
###
**4.** Chart a path to the destination bin
###
**5.** Maneuver the Kuka KR210 along the charted path
###
**6.** Sucessfully place the object in the destination bin
###
###

If you'd like to watch the Kuka KR210 in action, click [**here.**](https://youtu.be/rV0lWmJ7uOM)

Check out the Amazon Robotics Challenge [**here.**](https://youtu.be/yVIRLao1E28)

###

We operate the 6 D.o.F. arm through **ROS Kinetic** (ran through Linux Ubuntu 16.0) and commands are written in **Python**.

The code driving this project and interacting with ROS can be found at `IK_server.py`

*Quick note on naming convention:* `THIS_IS_A_CONSTANT` *and* `thisIsAVariable`

This **README** is broken into the following sections: **Environment Setup, Code Analysis, and Debugging**

###
###
###

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

Now from a terminal window, update the packages and set permissions for our executables:
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

Once **Gazebo** and **RViz** are up and running, make sure you see following in the Gazebo world: **KR210 Robot, Shelf, Blue cylindrical target on one of the shelves,and a Dropbox next to the robot**
    
Now, open a new terminal and type:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/
$ rosrun kuka_arm IK_server.py
```
*(If you're getting errors in the terminal window that you ran `rosrun kuka_arm IK_server.py` in, they are likely making the robot act unusual or fail at the task. Press `ctrl+c` in the terminal windows and launch `./safe_spawner.sh` and `rosrun kuka_arm IK_server.py` again.)*

Now, navigate to the **RViz** window and click `continue` to begin the motion of the KR210 arm.

###
###
###

### Code Analysis

`IK_server.py` houses our code that will calculate the joint variables for the Kuka KR210.  The KR210 has 6 *Revolute Type* joints. Counting up from the base link, we have: `theta1` `theta2` `theta3` `theta4` `theta5` `theta6`

<p align="center">
    <img src="https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Diagrams/l20-inverse-kinematics-02.png?raw=true">
</p>

The first 3 joints `theta1` `theta2` `theta3` have the largest impact on the location of the end effector, or gripper; while the last 3 joints `theta4` `theta5` `theta6` make up the *spherical wrist*, with `theta5` being the Wrist Center or `WC`.

<p align="center">
    For reference, a real Kuka KR210 looks like this:<br>
    <img src="https://media.robots.com/images/1363894998_1.jpg">
    <br><br>
    A quick refresher on joint types:
</p>

###

![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Joint%20Types/joint-types-and-degrees-of-freedom-01.png?raw=true)
Now that you have some background, it's time to dig in.

###
###

##### Denavit-Hartenberg (DH) Parameters
**DH** Parameters are worth mentioning here, as they are a very useful method for describing the configuration of reference frames. They use **4** parameters instead of the typical **6** parameters to describe position and orientation.

The **DH** method is now ubiquitous, but there are many conventions for using it. We'll be using the convention from ***John J. Craig***; referenced in his book, *"Craig, JJ. (2005). Introduction to Robotics: Mechanics and Control"*

###

The four parameters are: `alpha, a, d, theta` and are defined as:

![DH parameter definitions](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Denavit-Hartenberg%20Parameters/DH_Parameter_Definitions.png?raw=true)

###

And can be seen in the diagram below:

![DH parameters on Joints](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Denavit-Hartenberg%20Parameters/denavit-hartenberg-parameter-definitions-01.png?raw=true)

###

In order to determine our DH parameters, we need to assign reference frames to our serial manipulator. This is accomplished in an **8** step process as follows:

![DH parameter assignment](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Denavit-Hartenberg%20Parameters/DH_Parameter_Assignment_Process.png?raw=true)

###

The diagram below shows the differences between how reference frames are placed with the **DH-convention** *versus* the **URDF file**. The black triangles represent the placement of reference frame origins in the URDF file, while the red dots represent the placement using DH-convention.

As you can see, some reference frames are not placed at the same origins. We need to keep this in mind while filling out our DH parameter table.

###

![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Diagrams/Kuka_KR210_Orientation.png?raw=true)

###
###
###

##### URDF Analysis
We'll fill out the table using the diagram above, and the URDF file `kr210.urdf.xacro`, which can be found using terminal:

```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/urdf/
$ nano kr210.urdf.xacro
```
Lines `316-363` are of particular interest to us. Specifically, the  `origin xyz` lines. For example, in `joint1`:
###### Joint1
###
```
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
```
We interpret this as its location relative to its parent link, which in this case is the base link. 

This can be seen on line `324`: `<parent link="base_link"/>`. This is an important detail to keep in mind: ***measurements in the URDF file are relative to the prior joint (the parent link)***

So, this means `joint1` *(in URDF terms)* is at the same `X` and `Y` position of `joint1` - hence the first two zeros; and it is `0.33` above `joint1` in the positive `Z` direction. Look for this in **green** on the annotated Kuka KR210 diagram below.

###### Joint2
###
```
<joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
```
`joint2` relative to `joint1` is `0.35` ahead on the `x-axis`, at the same `Y` position, and is `0.42` above on the `z-axis`. This gives us `d1=0.75=(0.33+0.42)` on our **DH-parameter table:** *the signed distance from `x0` to `x1` along `z1`.*

We also obtain `a1=0.35` here, which is the distance from `z1` to `z2` along `x1`. Since our `origin x` measurement was 0.35 relative to `joint1`, we know this is correct.


###### Joint3
###
```
<joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
```
`joint3` relative to `joint2` is at the same `X` and `Y` position, and is `1.25` above on the `z-axis`. This actually gives us our `a2` as well as our `SIDE_C` measurement of the triangle we will draw and reference later on.


###### Joint4
###
```
<joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
```
`joint4` relative to `joint3` is `0.96` ahead on the `x-axis`, at the same `Y` position, and is `0.054` below on the `z-axis`*(notice the negative sign)*. This actually gives us our `SIDE_S` measurement of a triangle we draw on the diagram below and reference later on; the `-0.054` is our `a3` measurement, and the first component of `JOINTS_4AND5_X` is `0.96`.




###### Joint5
###
```
<joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
```
`joint5` relative to `joint4` is `0.54` ahead on the `x-axis`, and at the same `Y` and `Z` position. This actually gives us the second component of our `JOINTS_4AND5_X`, `0.54` `(JOINTS_4AND5_X = 1.5 = (0.96+0.54))`. A measurement of a triangle we draw on the diagram below and reference later on.



###### Joint6
###
```
<joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
```
`joint6` relative to `joint5` is `0.193` ahead on the `x-axis`, and at the same `Y` and `Z` position. This gives us the first component of `d7`. We now just have to add the gripper `X` position to it.

###### Gripper
###
`Line 288` gives us the `gripper joint` location of:
```
<origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
```
We then take the `0.193` and `0.11` and add them together to get `d7=0.303`

***With this information from the `kr210.urdf.xacro` file, we fill in our diagram:***
###
![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Diagrams/Kuka_KR210_Orientation_annotated.png?raw=true)

***And our DH Parameter Table:***
###
![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Denavit-Hartenberg%20Parameters/DH_parameter_table.JPG?raw=true)

###

###

##### Forward and Inverse Kinematics

In this project, we're trying to solve two interesting problems:

**1.** Forward Kinematics
###
**2.** Inverse Kinematics

###

![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Diagrams/forward-kinematics-01.png?raw=true)

###

##### Forward Kinematics (FK)
**FK** is what we use to determine where our end effector currently is.

<p align="center">
    <br> FK is the composition of Homogeneous Transforms (a simultaneous Rotation and Translation)
    <br><br>
    <img src="https://d17h27t6h515a5.cloudfront.net/topher/2017/June/5940a1d0_eq/eq.png">
    <br><br>
    <br> The total Transform between Links:
    <br><br>
    <img src="https://d17h27t6h515a5.cloudfront.net/topher/2017/June/593eebcd_eq1/eq1.png">
    <br> Each link is composed of two rotations and two translations, performed in this order:
    <br><br>
    <img src="https://d17h27t6h515a5.cloudfront.net/topher/2017/June/593eecf9_eq2/eq2.png">
</p>

###

##### Inverse Kinematics (IK) 
**IK** is the more challenging of the two. With IK, we are given the position of the target object in *Cartesian Form (x,y,z)* and we have to determine which configuration we should choose for `theta1-6` (the robot's joints). This is no easy task, and often has more than one solution, or even no solutions!

###

To complete our Forward and Inverse Kinematics problems, we'll harness the power of a Computer Algebra System(CAS) for Python, called **Sympy**.

###

![alt text](https://d17h27t6h515a5.cloudfront.net/topher/2017/June/5937535e_sympy-logo.svg/sympy-logo.svg.png)
*I would encourage the interested reader to check out their helpful documentation [**here.**](http://docs.sympy.org/latest/index.html)*

**Sympy** allows us to use symbols and build equations without evaluating them. For instance we can create some symbols that will act as our joint angles: 
```
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
```
We can then build matrices with those symbols. A homogeneous transform(rotation and translation), from **Forward Kinematics**, from frame 0-1 is:
```
T0_1 = Matrix([[cos(q1), -sin(q1),  0, a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [      0,    0,    0,    1]])
```

We also need to correct for discrepancies between frames in **Gazebo** by `180deg` about the `z-axis`, and `-90deg` about the `y-axis`. So we build a correction Matrix `R_corr`:
```
R_corr = rot_z.subs(y, radians(180)) * rot_y.subs(p, radians(-90))
```
where `rot_z` and `rot_y` are simply rotation matrices:
```
rot_z = Matrix([[ cos(y),   -sin(y),        0],
                [ sin(y),    cos(y),        0],
                [ 0,              0,        1]])
# Rotation about Z axis (Yaw)

rot_y = Matrix([[  cos(p),        0,   sin(p)],
                [       0,        1,        0],
                [ -sin(p),        0,   cos(p)]])
# Rotation about Y axis (Pitch)
```
We then define `ROT_G` which is a composition of Rotations:
```
ROT_G = rot_z * rot_y * rot_x 
```
and then correct for the discrepancies in **Gazebo**:
```
ROT_G = ROT_G * R_corr
```
###
Our program spits out the position of the end effector as `px`, `py`,`pz`, and we create an end effector Matrix `EE`:
```
EE = Matrix([[px],
             [py],
             [pz]])
```
Evaluate the Rotation Matrix at the `roll`, `pitch`, and `yaw` values given to us:
```
ROT_G = ROT_G.subs({'r':roll, 'p':pitch,'y':yaw})
```

###
We can determine our Wrist Center `WC` using the location of the end effector, our `d7` from the DH parameter table , and our Rotation Matrix `ROT_G` from above.
```
WC = EE - 0.303 * ROT_G[:,2]
Wx = WC[0]
Wy = WC[1]
Wz = WC[2]
```
Returning to our Diagram from earlier...
###
![alt text](https://github.com/chriswernst/Amazon-Challenge-Udacity-RoboticsND-Project2/blob/master/Forward%20and%20Inverse%20Kinematics/Diagrams/Kuka_KR210_Orientation_annotated.png?raw=true)
###
...we can start determining our `thetas`:

###### theta1
`theta1` is the first joint angle, and we can determine it using Arc Tangent, which in python is `atan2`:
```
theta1 = atan2(Wy, Wx)
```
###

###### theta2
`theta2` is far more challenging, so let's get going. We need to draw some triangles, and define their sides and angles. We'll use these triangles to help define the rest of the `thetas`. Reference the annotated diagram to see where the sides and angles are located.
```
JOINTS_4AND5_X = 1.5 # (0.96 + 0.54)
# This is taken from line 344, 351(joints 4 and 5) of the URDF file 'kr210.urdf.xacro'. This is a constant.

# SIDES
SIDE_S = 0.054
# Taken from the Z measurement from the URDF (joints 4 and 5), lines 344, 355. 
SIDE_A = sqrt(SIDE_S**2 + JOINTS_4AND5_X**2)
sideB = sqrt((Wz-0.75)**2 + (sqrt(Wx**2 + Wy**2) -0.35)**2)          
SIDE_C = 1.25

# ANGLES
# We're going to leverage the Law of Cosines: c**2 = a**2 + b**2 - 2*a*b*cos(C) -> where C is the desired angle
angleA = acos((SIDE_C**2 + sideB**2 - SIDE_A**2)/(2*SIDE_C*sideB))
angleB = acos((SIDE_C**2 + SIDE_A**2 - sideB**2)/(2*SIDE_C*SIDE_A))
angleC = acos((SIDE_A**2 + sideB**2 - SIDE_C**2)/(2*SIDE_A*sideB))

angleG = atan2((Wz - 0.75), sqrt((Wx-0.35)**2 + Wy**2))
# I've defined 'angleG' as the angle from joint2 pointing along the X axis
    # where the hypoteneuse hits the WC. Theta2(or q2) plus angleG, plus angleA
    # equals pi/2, a right angle.
    
ANGLE_S = atan2(SIDE_S, JOINTS_4AND5_X)
# I've defined 'ANGLE_S' as the angle above SIDE_A that makes angleB into a 90deg angle
    # when the Kuka KR210 is in its zero configuration. This is a constant.
    # Note, this could also have been done with 'asin(SIDE_S/SIDE_A)'

```
###
###
We can now define `theta2` since we know that `theta2 + angleA + angleG = 90deg`:
```
theta2 = np.pi/2. - angleG - angleA     
```         
###
###### theta3
###
Since we've drawn our triangles already, we can leverage what we've defined, and solve for `theta3`:
```
theta3 = np.pi/2. - ANGLE_S - angleB    
```
###
###### theta4, theta5, theta6
###
`theta4-6` will be determined by exploiting the `Euler Angles from a Rotation Matrix` lesson. 
We want to isolate the final 3 `thetas`, so we'll solve for what we know. The first three Rotations from the base frame to frame3. Reminder: `R0_3 = R0_1 * R1_2 * R2_3`
```
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
```
Earlier, we solved for the complete Rotation from frame0 to the gripper, with `ROT_G` 
```
ROT_G = R0_6
```
We want to isolate `R3_6`. So, using handy properties from algebra:
```
R3_6 = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6  / (R0_1 * R1_2 * R2_3)
```
`R0_3` cancels on the **RHS** of the equation, and we're left with:
```
R3_6 = R0_3.T * ROT_G
```
`R3_6` is the matrix we're going to extract our `theta4-6` from:

Recall that:
###
![alt text](https://d17h27t6h515a5.cloudfront.net/topher/2017/August/5981edf6_extrinsicxyz/extrinsicxyz.png)
###
Where we learned that we could find `Beta` with:
###
![alt text](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e122e_image-1/image-1.png)
###
And
###
![alt text](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e166f_codecogseqn-3/codecogseqn-3.gif)
![alt text](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e15e2_codecogseqn-2/codecogseqn-2.gif)
###

We're going to use some creative trigonometry rules to isolate the `thetas` and get the rest of the arguments to cancel out:
```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```

###### And that's it! We have officially generated `thetas1-6` from only a final end-effector position - We have solved Inverse Kinematics!
###
###


### Debugging
If you're running on a Virtual Machine, you might find that your KR210's motions are correct, but the gripper is not completely grasping the cylindrical targets. This can be solved by putting a delay between the steps. Open a terminal and type:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/src/
$ nano trajectory_sampler.cpp
```
edit `line 327` of `trajectory_sampler.cpp` to say:
```cpp
ros::Duration(2).sleep();
```
You may want to try anywhere from `1.o` second delay all the way to `5` depending on how slow the VM is running.




