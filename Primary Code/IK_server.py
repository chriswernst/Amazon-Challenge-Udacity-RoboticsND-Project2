#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# NOTE: This is Python2 Code!

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# This is the location for our Forward and Inverse Kinematics code:
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
                        		
        ##### FK code here

        ### Create symbols
        
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        # The Thetas (joint angle - angle between Xi-1 to Xi), and one for the gripper
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        # (link offset - signed distance from Xi-1 to Xi measured along Zi)
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        # (Link length - distance from Zi-1 to Zi, measured along Xi-1 where Xi-1 is perpendicular to both Zi-1 and Zi)
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        # (Twist angle - angle between Zi-1 and Z- measured about Xi-1 in a right hand sense)
        
        
        ### Create Modified DH parameters
        
        # Parameters for KUKA KR210 #
        
        # This information is located in the URDF files
        # URDF Folder > kr210.urdf.xacro
        
        # Create a dictionary of DH Parameters
        DH_TABLE = {alpha0:     0,     a0:      0,    d1: 0.75,    q1:          q1,
                    alpha1: -pi/2,     a1:   0.35,    d2: 0,       q2:   q2 - pi/2, 
                    alpha2:     0,     a2:   1.25,    d3: 0,       q3:          q3,
                    alpha3: -pi/2,     a3: -0.054,    d4: 1.5,     q4:          q4,
                    alpha4:  pi/2,     a4:      0,    d5: 0,       q5:          q5,
                    alpha5: -pi/2,     a5:      0,    d6: 0,       q6:          q6,
                    alpha6:     0,     a6:      0,    d7: 0.303,   q7:           0}

                
        
        def transformationMatrix(alpha, a, d, q):
            TF = Matrix([[             cos(q),            -sin(q),            0,       a],
                      [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                      [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                      [                   0,                   0,            0,       1]])
            return TF
        
        # Homogeneous Transforms for between neighboring links: 
        # Create individual transformation matrices

        T0_1 = transformMatrix(alpha0, a0, d1, q1).subs(DH_TABLE) # Base Link to Link1
        T1_2 = transformMatrix(alpha1, a1, d2, q2).subs(DH_TABLE) # Link1 to Link2
        T2_3 = transformMatrix(alpha2, a2, d3, q3).subs(DH_TABLE) # Link2 to Link3
        T3_4 = transformMatrix(alpha3, a3, d4, q4).subs(DH_TABLE) # Link3 Link to Link4
        T4_5 = transformMatrix(alpha4, a4, d5, q5).subs(DH_TABLE) # Link4 to Link5       
        T5_6 = transformMatrix(alpha5, a5, d6, q6).subs(DH_TABLE) # Link5 to Link6       
        T6_G = transformMatrix(alpha6, a6, d7, q7).subs(DH_TABLE) # Link6 to Gripper             
        
        # Composition of Homogeneous Transforms 

        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
        # this is the transformation matrix from the base link to the end effector
        
              
        def rot_x(q):
            
            R_x = Matrix([[ 1,             0,         0,    0],
                          [ 0,        cos(q),   -sin(q),    0],
                          [ 0,        sin(q),    cos(q),    0],
                          [ 0,             0,         0,    1]])
                    
            return R_x
            # Rotation about X axis (Roll)
        
            
        def rot_y(q):              
            
            R_y = Matrix([[  cos(q),        0,   sin(q),    0],
                          [       0,        1,        0,    0],
                          [ -sin(q),        0,   cos(q),    0],
                          [       0,        0,        0,    1]])
        
            return R_y
            # Rotation about Y axis (Pitch)
        
        
        def rot_z(q):    
        
            R_z = Matrix([[ cos(q),   -sin(q),        0,    0],
                          [ sin(q),    cos(q),        0,    0],
                          [ 0,              0,        1,    0],
                          [ 0,              0,        0,    1]])
            
            return R_z
            # Rotation about Z axis (Yaw)
            
        
        
	    ### Compensate for rotation discrepancy between DH parameters and Gazebo
        
        R_z = rot_z(pi)
        # Rotate about the Z axis by 180deg
        R_y = rot_y(-pi/2)
        # Rotate about the Y axis by -90deg   
        R_corr = R_z * R_y
        
        T_total = T0_G * R_corr
        # This line also takes time to calculate (~15s)
        
        # Evaluate numerically at '0' to verify FK section with Rviz (NB pg. 84 - 'Debugging FK')
        
        # gripper_testing = print(T_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
            # correct at d1=.75, d2=0
	           
        
  
            
        ##### Your IK code here 

	    # Extract end-effector position and orientation from request
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
        
        	    # px,py,pz = end-effector position
             # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
    
        	   # Compensate for rotation discrepancy between DH parameters and Gazebo

            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
            # Now need to extract Nx, Ny, Nz from Rrpy matrix to give us the WC position
            
            Nx = Rrpy[0,2]
            Ny = Rrpy[1,2]
            Nz = Rrpy[2,2]
            
            Wx = px - .303 * Nx
            Wy = py - .303 * Ny
            Wz = pz - .303 * Nz
            
            WC = Matrix([[Wx],
                         [Wy],
                         [Wz]])
            
            theta1 = atan2(Wy, Wx)  
            # theta1
            
            
            
            # Now, let's determine theta2:
            
            SIDE_S = 0.054
            # Taken from the Z measurement from the URDF (joints 4 and 5), lines 344, 355. This is a constant.
            
            JOINTS_4AND5_X = 0.96 + 0.54
            # This is taken from line 344, 351(joints 4 and 5) of the URDF file 'kr210.urdf.xacro'. This is a constant.
            
            # SIDES
            SIDE_A = sqrt(SIDE_S**2 + JOINTS_4AND5_X**2)
            # SIDE_A is a constant length
            sideB = sqrt((Wz-0.75)**2 + (Wx-0.35)**2 + Wy**2)
            # sideB is a variable length
            SIDE_C = 1.25
            # SIDE_C is a constant length
            
            # ANGLES
            # We're going to leverage the Law of Cosines: c**2 = a**2 + b**2 - 2*a*b*cos(C) -> where C is the desired angle
            angleA = acos((SIDE_C**2 + sideB**2 - SIDE_A**2)/(2*SIDE_C*sideB))
            # angleA is a variable angle
            angleB = acos((SIDE_C**2 + SIDE_A**2 - sideB**2)/(2*SIDE_C*SIDE_A))
            # angleB is a variable angle            
            angleC = acos((SIDE_A**2 + sideB**2 - SIDE_C**2)/(2*SIDE_A*sideB))
            # angleC is a variable angle            
            
            angleG = atan2((Wz - 0.75), sqrt((Wx-0.35)**2 + Wy**2))
            # I've defined 'angleG' as the angle from joint2 pointing along the X axis
                # where the hypoteneuse hits the WC. Theta2(or q2) plus angleG, plus angleA
                # equals pi/2, a right angle.
            
            # We can now define theta2 since we know that theta2 + angleA + angleG = 90deg; therefore:
            
            theta2 = pi/2 - angleG - angleA     
            #theta2


            # Now, let's determine theta3:
            
            # We'll use the same logic from theta2 of defining a 90degree angle, then deducing theta3 from that.
            
            ANGLE_S = atan2(SIDE_S, JOINTS_4AND5_X)
            # I've defined 'ANGLE_S' as the angle above SIDE_A that makes angleB into a 90deg angle
                # when the Kuka KR210 is in its zero configuration. This is a constant.
                # Note, this could also have been done with 'asin(SIDE_S/SIDE_A)'

            theta3 = pi/2 - ANGLE_S - angleB     
            # theta3
            
            # Use tranpose instead of inv("LU")
            
            R0_6 = Rrpy
            
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            # this cuts our homogeneous transforms down to 3x3 rotation matrices
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            
            R3_6 = R0_3.T * Rrpy
            
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
            
            FK = T_total.evalf(subs{q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6:theta6})
                
            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
    