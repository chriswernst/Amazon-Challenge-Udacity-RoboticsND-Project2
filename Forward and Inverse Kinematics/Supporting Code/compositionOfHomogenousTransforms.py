#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 27 08:57:09 2017

@author: ChrisErnst

NOTE: This is a composition of homogeneous transforms, WITHOUT considering and end effector.
Essentially, it is many 4x4 matrices(homogeneous transforms) multiplied together, without the final 4x1 vector that
typically represents the end effector
"""

import numpy as np
import os
from sympy import symbols, cos, sin, simplify, pi
from sympy.matrices import Matrix

os.chdir('/Robotics Nanodegree - Udacity/Term 1/3. Kinematics (Project 2)/2. Forward and Inverse Kinematics')
open('composition-of-homogeneous-transforms-01.png', mode='r')
# This is the accompanying image to this exercise. Note, this will not open it...



### Create symbols for joint variables
# The numbers 1 to 4 correspond to each rotation in the order specified to you.
q1, q2, q3, q4 = symbols('q1:5')


### Define functions for Rotation Matrices about x, y, and z given specific angle.

def rot_x(q):
    
    R_x = Matrix([[ 1,             0,         0],
                  [ 0,        cos(q),   -sin(q)],
                  [ 0,        sin(q),   cos(q)]])
    
    # R_x = R_x.evalf(subs={q1:q1*dtr})

    return R_x
    # Rotation about X axis

    
def rot_y(q):              
    
    R_y = Matrix([[  cos(q),        0,   sin(q)],
                  [       0,        1,        0],
                  [ -sin(q),        0,  cos(q)]])
    
    # R_y = R_y.evalf(subs={q2:q2*dtr})

    return R_y
    # Rotation about Y axis


def rot_z(q):    

    R_z = Matrix([[ cos(q), -sin(q),        0],
                  [ sin(q),  cos(q),        0],
                  [ 0,              0,        1]])
    
    # R_z = R_z.evalf(subs={q3:q3*dtr})
    
    return R_z
    # Rotation about Z axis



### Define rotations between frames

# Initial Rotation Matrix for Frame A
Ra = Matrix([[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]])
    # Same output as rot_x(0), rot_y(0), or rot_z(0)

    
# Rotations performed on individual Frames for A->B->E
Rb_a = rot_y(q1)
Re_b = rot_x(q2) 

# Rotations performed on individual Frames for A->C->D->E
Rc_a = Ra
Rd_c = rot_x(q3)
Re_d = rot_z(q4)



### Define Translations between frames.

# Route one A->B->E
tb_a = Matrix([[-2],
                [2],
                [4]])

te_b =  Matrix([[0],
                [2],
                [0]])

    
# Route two A->C->D->E
tc_a = Matrix([[4],
               [4],
               [0]])

td_c = Matrix([[-3],
                [3],
                [2]])

te_d = Matrix([[-3],
                [2],
                [3]])


    
### Now, combine the Rotations and Translations into a 4x4 matrix 
    # which we call a homogenous transformation
    
bottomRow = Matrix([[0,0,0,1]])
# Generate the bottom Row of the Homogenous Transform Matrix

# Route one A->B->E
Ta = Ra.row_join(Matrix([[0],  
                         [0],
                         [0]])).col_join(bottomRow)

Tb_a = Rb_a.row_join(tb_a).col_join(bottomRow)

Te_b = Re_b.row_join(te_b).col_join(bottomRow)

# Route two A->C->D->E
Tc_a = Rc_a.row_join(tc_a).col_join(bottomRow)

Td_c = Rd_c.row_join(td_c).col_join(bottomRow)

Te_d = Re_d.row_join(te_d).col_join(bottomRow)


### Composition of Transformations
Te_a_1 = simplify(Ta * Tb_a * Te_b)

Te_a_2 = simplify(Ta * Tc_a * Td_c * Te_d)



### Calculate orientation and position for E
E_1 = Te_a_1.evalf(subs={q1: -pi/2, q2: pi/2}, chop = True)

E_2 = Te_a_2.evalf(subs={q3: pi/2, q4: pi/2}, chop = True)

print("Transformation Matrix for A->B->E:")
print(E_1)

print("Transformation Matrix for A->C->D->E:")
print(E_2)
# This proves that both matrices are equivalent. You can use either route 
# to reach point E!

os.chdir('/Robotics Nanodegree - Udacity/Term 1/3. Kinematics (Project 2)/2. Forward and Inverse Kinematics')
open('composition-of-homogeneous-transforms-01.png', mode='r')
# This is the accompanying image to this exercise
# Again, note that there is not an end effector in the exercise, so we do not 
    # execute final matrix multiplication of the homogeneous transform(4x4) and 
    # the end effector, Point P (4x1)










############# BELOW IS SIMPLY FOR REFERENCE #####################


R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(q1), -sin(q1)],
              [ 0,        sin(q1),  cos(q1)]])
# Rotation about X axis

R_y = Matrix([[ cos(q2),        0,  sin(q2)],
              [       0,        1,        0],
              [-sin(q2),        0,  cos(q2)]])
# Rotation about Y axis

R_z = Matrix([[ cos(q3), -sin(q3),        0],
              [ sin(q3),  cos(q3),        0],
              [ 0,              0,        1]])
# Rotation about Z axis

offset =  Matrix([[1],
                  [0],
                  [30]])
# Offset


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


############# ABOVE IS SIMPLY FOR REFERENCE #####################



