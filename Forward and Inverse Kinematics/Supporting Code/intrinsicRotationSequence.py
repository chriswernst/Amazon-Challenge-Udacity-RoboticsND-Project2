#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 14:59:48 2017

@author: ChrisErnst
"""
# This code shows how an intrinsic rotation sequence about the Y then Z axes is performed
# REMINDER: Intrinsic Rotations are done with Post Multiplication

import numpy as np
from sympy import symbols, cos, sin, pi, sqrt
from sympy.matrices import Matrix

# Conversion Factors
rtd = 180./np.pi # radians to degrees
dtr = np.pi/180. # degrees to radians

### Create symbols for joint variables
q1, q2 = symbols('q1:3')


# Create a symbolic matrix representing an intrinsic sequence of rotations 
  # about the Y and then Z axes. Let the rotation about the Y axis be described
  # by q1 and the rotation about Z by q2. 
  


# Rotation about Y axis
R_y = Matrix([[ cos(q1),        0,  sin(q1)],
              [       0,        1,        0],
              [-sin(q1),        0,  cos(q1)]])

# Rotation about Z axis
R_z = Matrix([[ cos(q2), -sin(q2),        0],
              [ sin(q2),  cos(q2),        0],
              [ 0,              0,        1]])

YZ_intrinsic_sym = R_y * R_z


# Numerically evaluate YZ_intrinsic assuming:
   # q1 = 45 degrees and q2 = 60 degrees. 
   # NOTE: Trigonometric functions in Python assume the input is in radians! 


YZ_intrinsic_num = YZ_intrinsic_sym.evalf(subs={q1: 45*dtr, q2: 60*dtr})



