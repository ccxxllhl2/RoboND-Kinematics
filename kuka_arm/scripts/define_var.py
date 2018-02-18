#!/usr/bin/env python

# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program

# Author: Nick Zanobini

import numpy as np
from sympy import *

### These func will be called ###
def rot_x(angle):
    rx =   Matrix([[ 1,         0,            0],
                   [ 0, cos(angle), -sin(angle)],
                   [ 0, sin(angle),  cos(angle)]])
    return rx
    
def rot_y(angle):
    ry =   Matrix([[ cos(angle),  0, sin(angle)],
                   [          0,  1,          0],
                   [-sin(angle),  0, cos(angle)]])
    return ry   
                
def rot_z(angle):
    rz =   Matrix([[  cos(angle), -sin(angle), 0],
                   [  sin(angle),  cos(angle), 0],
                   [      0,        0,         1]])
    return rz

def homo_tf(alpha, a, d, q):
    TM =   Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])
    return(TM)
### ************************ ###

# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Create Modified DH parameters
s = {alpha0:      0, a0:       0, d1:   0.75,
     alpha1:  -pi/2, a1:    0.35, d2:      0, q2: q2-pi/2,
     alpha2:      0, a2:    1.25, d3:      0,
     alpha3:  -pi/2, a3:  -0.054, d4:    1.5,
     alpha4:   pi/2, a4:       0, d5:      0,
     alpha5:  -pi/2, a5:       0, d6:      0,
     alpha6:      0, a6:       0, d7:  0.303, q7:0}

# Create individual transformation matrices
T0_1 = homo_tf(alpha0, a0, d1, q1)
T0_1 = T0_1.subs(s)

T1_2 = homo_tf(alpha1, a1, d2, q2)
T1_2 = T1_2.subs(s)

T2_3 = homo_tf(alpha2, a2, d3, q3)
T2_3 = T2_3.subs(s)

T3_4 = homo_tf(alpha3, a3, d4, q4)
T3_4 = T3_4.subs(s)

T4_5 = homo_tf(alpha4, a4, d5, q5)
T4_5 = T4_5.subs(s)

T5_6 = homo_tf(alpha5, a5, d6, q6)
T5_6 = T5_6.subs(s)

T6_G = homo_tf(alpha6, a6, d7, q7)
T6_G = T6_G.subs(s)

### These var will be called ###
T0_3 = T0_1 * T1_2 * T2_3
R_corr = rot_z(pi) * rot_y(-pi / 2)
R0_3_inv = T0_3[0:3, 0:3].transpose()
a1 = 0.35
a2 = 1.25
a3 = -0.054
d1 = 0.75
d4 = 1.5
d7 = 0.303
### ************************ ###
