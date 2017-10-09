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
        s = {alpha0:     0,     a0:      0,    d1: 0.75,    q1:          q1,
             alpha1: -pi/2,     a1:   0.35,    d2: 0,       q2:   q2 - pi/2, 
             alpha2:     0,     a2:   1.25,    d3: 0,       q3:          q3,
             alpha3: -pi/2,     a3: -0.054,    d4: 1.5,     q4:          q4,
             alpha4:  pi/2,     a4:      0,    d5: 0,       q5:          q5,
             alpha5: -pi/2,     a5:      0,    d6: 0,       q6:          q6,
             alpha6:     0,     a6:      0,    d7: 0.303,   q7:           0}

                
        # Homogeneous Transforms for between neighboring links: 
            # Define Modified DH Transformation matrix 	 	
            # Create individual transformation matrices
            # Extract rotation matrices from the transformation matrices
        
        
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)
            # For between Link 0 and 1
        
        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)
                # For between Link 1 and 2
                        
        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)
                # For between Link 2 and 3
                
        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)
                # For between Link 3 and 4
                
        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)
                # For between Link 4 and 5
                
        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)
                # For between Link 5 and 6
                      
        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_G = T6_G.subs(s)
                # For between Link 6 and Gripper (7)
        

        # Composition of Homogeneous Transforms
            # Transform from base link to end effector; use simplify to simplify trig expressions.
        T0_2 = simplify(T0_1 * T1_2) # Base Link to Link2
        T0_3 = simplify(T0_2 * T2_3) # Base Link to Link3
        T0_4 = simplify(T0_3 * T3_4) # Base Link to Link4
        T0_5 = simplify(T0_4 * T4_5) # Base Link to Link5
        T0_6 = simplify(T0_5 * T5_6) # Base Link to Link6
        T0_G = simplify(T0_6 * T6_G) # Base Link to Gripper
        # Note: this block of code takes time to execute(~30s)
        
            
	    ### Compensate for rotation discrepancy between DH parameters and Gazebo
        
        R_z = Matrix([[cos(pi),   -sin(pi),       0,          0],
                    [  sin(pi),    cos(pi),       0,          0],
                    [        0,          0,       1,          0],
                    [        0,          0,       0,          1]])    
                # Rotate about the Z axis by 180deg
        
        R_y = Matrix([[cos(-pi/2),         0,  sin(-pi/2),    0],
                    [           0,         1,           0,    0],
                    [ -sin(-pi/2),         0,  cos(-pi/2),    0],
                    [           0,         0,           0,    1]])    
                # Rotate about the Y axis by -90deg   
        R_corr = simplify(R_z * R_y)
        
     
        
        T_total = simplify(T0_G * R_corr)
        # This line also takes time to calculate (~15s)
        
        # Evaluate numerically at '0' to verify FK section with Rviz (NB pg. 84 - 'Debugging FK')
        
        # gripper_testing = print(T_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
            # correct at d1=.75, d2=0
	           
        
        
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
    