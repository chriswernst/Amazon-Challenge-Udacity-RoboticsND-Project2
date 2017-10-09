#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 26 10:21:34 2017

@author: ChrisErnst
"""

from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

###############################################################
# Problem Statement:
  # Let P be a vector expressed in frame {B} with (x,y,z)
  # coordinates = (15.0, 0.0, 42.0)
  # Rotate P about the Y-axis by angle = 110 degrees (20 + 90deg)
  # Then translate the vector 1 unit
  # in the X-axis and 30 units in the Z-axis. 
  # Print the new (x, y, z) coordinates of P after the transformation.  
###############################################################

#### Create symbols for joint variables
q2 = symbols('q2')
gamma  = symbols('gamma')

# Conversion Factors
rtd = 180./np.pi # radians to degrees
dtr = np.pi/180. # degrees to radians

# The operations we're going to need to do:
    
    # [4,1]   = [4,4] * [4,1]
    # [P_new] = [T_y] * [ P ]
    # [pointP relative to A] = [homegenous transform: rotation matrix(3x3) and offset(3x1)] * [location of P relative to B]



#### TO DO ####
# Replace P and T with appropriate expressions and calculate new coordinates of P in {A}. 
P = Matrix([[15],
            [0],
            [42],
            [1]])
# P is the location of point P(the end effector), relative to B. Should be a 4x1 Matrix

offset =  Matrix([[1],
                  [0],
                  [30]])

T_y_sym = Matrix([[cos(q2),        0,  sin(q2),   offset[0]],
                [       0,         1,        0,   offset[1]],
                [-sin(q2),         0,  cos(q2),   offset[2]],
                [       0,         0,        0,          1]]) 
    
# T_y_sym Should be a 4x4 homogeneous Transform. A Rotation about the Y Axis
# This is a rotational matrix (3x3) + offset matrix(3x1), with some added values so we can multiply (bottom row)


T = T_y_sym.evalf(subs={q2:110*dtr})


P_new = T * P # 
# P_new (4x1) is the location of point P(end effector), relative to A



################### BELOW IS SIMPLY FOR REFERENCE #########################


T_x = Matrix([[ 1,            0,         0,   offset[0]],
              [ 0,      cos(q1),  -sin(q1),   offset[1]],
              [ 0,      sin(q1),   cos(q1),   offset[2]],
              [ 0,            0,           0,       1]])    
# Homogenous Transform about the X axis

T_y = Matrix([[cos(q2),        0,  sin(q2),   offset[0]],
            [       0,         1,        0,   offset[1]],
            [-sin(q2),         0,  cos(q2),   offset[2]],
            [       0,         0,        0,          1]])    
# Homogenous Transform about the Y axis

T_z = Matrix([[cos(q3),   -sin(q3),       0,  offset[0]],
            [  sin(q3),    cos(q3),       0,  offset[1]],
            [        0,          0,       1,  offset[2]],
            [        0,          0,       0,         1]])    
# Homogenous Transform about the Z axis



def rot_x(q1):
    
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,        cos(q1), -sin(q1)],
                  [ 0,        sin(q1),  cos(q1)]])
    
    R_x = R_x.evalf(subs={q1:q1*dtr})

    return R_x
    # Rotation about X axis

    
def rot_y(q2):              
    
    R_y = Matrix([[ cos(q2),        0,  sin(q2)],
                  [       0,        1,        0],
                  [-sin(q2),        0,  cos(q2)]])
    
    R_y = R_y.evalf(subs={q2:q2*dtr})

    return R_y
    # Rotation about Y axis


def rot_z(q3):    

    R_z = Matrix([[ cos(q3), -sin(q3),        0],
                  [ sin(q3),  cos(q3),        0],
                  [ 0,              0,        1]])
    
    
    
    R_z = R_z.evalf(subs={q3:q3*dtr})
    
    return R_z
    # Rotation about Z axis
