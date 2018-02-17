#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

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

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        # Create Modified DH parameters
        s = {alpha0:        0, a0:      0, d1: 0.75,
             alpha1:     -pi/2, a1:   0.35, d2:    0, q2: q2-pi/2,
              alpha2:        0, a2:     1.25, d3:    0,
               alpha3:     -pi/2, a3: -0.054, d4:  1.5,
             alpha4:     pi/2, a4:         0, d5:      0,
             alpha5:    -pi/2, a5:      0, d6:      0,
             alpha6:        0, a6:      0, d7:0.303, q7:0}
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

        T0_3 = simplify(T0_1 * T1_2 * T2_3)


        # Initialize service response
        
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            P_ee = Matrix([[px],[py],[pz]])
            R_corr = rot_z(pi) * rot_y(-pi / 2)
            R_z = rot_z(yaw)
            R_y = rot_y(pitch)
            R_x = rot_x(roll)
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)* R_corr
            P_wc = P_ee - Rrpy * Matrix([[0],[0],[s[d7]]])
            Wc_x, Wc_y, Wc_z =  P_wc[0], P_wc[1] , P_wc[2]

            # Finding theta 1-3

            # Implementing law of cosines to evaluate angles as shown in the figure in the Readme
            # theta1
            theta1 = atan2(Wc_y,Wc_x)
            
            # some lines about theta2-3
            s1 = sqrt(Wc_x**2 + Wc_y**2) - s[a1]
            s2 = Wc_z - s[d1]
            s3 = sqrt(s1**2 + s2**2)
            s4 = sqrt(s[d4]**2 + s[a3]**2)
            
            # theta2
            beta1 = atan2(s2, s1)
            cos_for_beta2 = (s[a2]**2 + s3**2 - s4**2) / (2*s[a2]*s3)
            beta2 = atan2(sqrt(1 - cos_for_beta2**2), cos_for_beta2)
            theta2 = pi/2 - beta2 - beta1
            
            # theta3          
            cos_for_beta3 = (s[a2]**2 + s4**2 -s3**2) / (2*s[a2]*s4)
            beta3 = atan2(sqrt(1-cos_for_beta3**2), cos_for_beta3)
            beta4 = atan2(-s[a3], s[d4])
            theta3 = pi/2 - beta3 - beta4
            
            
            # Finding theta 4-6

            R0_3 = T0_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R0_3_INV = R0_3.inv("LU")
            R3_6 = R0_3_INV * Rrpy
            r12 = R3_6[0, 1]
            r13 = R3_6[0, 2]
            r21 = R3_6[1, 0]
            r22 = R3_6[1, 1]
            r23 = R3_6[1, 2]
            r32 = R3_6[2, 1]
            r33 = R3_6[2, 2]

            theta5 = atan2(sqrt(r13**2 + r33**2), r23)
            if sin(theta5) < 0:
                theta4 = atan2(-r33, r13)
                theta6 = atan2(r22, -r21)
            else:
                theta4 = atan2(r33, -r13)
                theta6 = atan2(-r22, r21)

            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
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
