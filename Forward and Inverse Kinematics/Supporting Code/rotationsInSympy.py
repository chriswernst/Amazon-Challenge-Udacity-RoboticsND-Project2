#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 22 14:24:46 2017

@author: ChrisErnst
"""

from sympy import symbols, cos, sin, pi, simplify, eye, zeros, ones, diag
from sympy.matrices import Matrix
import numpy as np


# We construct them using the Sympy 'MATRIX' function




# Next, we define symbols that we will use in the rotation matrix.
# You can define a sequence of symbols, as:

### Create symbols for joint variables
q1, q2, q3, q4 = symbols('q1:5') # remember slices do not include the end value 
# unrelated symbols can be defined like this:
A, R, O, C = symbols('A R O C')


# Conversion Factors
rtd = 180./np.pi # radians to degrees
dtr = np.pi/180. # degrees to radians

# Now we create the rotation matrices for elementary rotations about the X, Y, and Z axes, respectively.

# The about Z rotation matrix is derived on page 45


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
    
# Numerically Evaluate the matrices 

print("Rotation about the X-axis by 45-degrees")
print(R_x.evalf(subs={q1: 45*dtr}))
print("Rotation about the y-axis by 45-degrees")
print(R_y.evalf(subs={q2: 45*dtr}))
print("Rotation about the Z-axis by 30-degrees")
print(R_z.evalf(subs={q3: 30*dtr}))
# Evaluates as a floating point number


    
    
'''
        ##### BEGIN Sympy Notes #####

# examples:

a = Matrix([[1,1,2],[3,4,5],[6,7,8]])


# colun vector:
colVec = Matrix([1,2,3,4,5])

# returns the shape of the matric
a.shape
colVec.shape

# returns the selected row or column
a.row(0)

a.col(0)

# inverse of a matrix
a**-1

# Transpose of a matrix
a.T

# Create the identity matrix
eye(3)

# Create an empty matrix
zeros(3,2)

# Create a matrix with ones
ones(4,2)

# Creates a square matrix with only the diagonal elements filled in
diag(2,2,2,2,2)

# To compute the determinant, use det
M = Matrix([[1, 0, 1], [2, -1, 3], [4, 3, 2]])

M.det()

        ##### END Sympy Notes #####

'''
