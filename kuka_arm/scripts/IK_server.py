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
from define_var import *
import numpy as np

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
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
            R_z = rot_z(yaw)
            R_y = rot_y(pitch)
            R_x = rot_x(roll)
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)* R_corr
            P_wc = P_ee - Rrpy * Matrix([[0],[0],[d7]])
            Wc_x, Wc_y, Wc_z =  P_wc[0], P_wc[1] , P_wc[2]

            # Finding theta 1-3
            # theta1
            theta1 = atan2(Wc_y,Wc_x)
            
            # some lines about theta2-3
            s1 = sqrt(Wc_x**2 + Wc_y**2) - a1
            s2 = Wc_z - d1
            s3 = sqrt(s1**2 + s2**2)
            s4 = sqrt(d4**2 + a3**2)
            
            # theta2
            beta1 = atan2(s2, s1)
            cos_for_beta2 = (a2**2 + s3**2 - s4**2) / (2*a2*s3)
            beta2 = atan2(sqrt(1 - cos_for_beta2**2), cos_for_beta2)
            theta2 = pi/2 - beta2 - beta1
            
            # theta3          
            cos_for_beta3 = (a2**2 + s4**2 -s3**2) / (2*a2*s4)
            beta3 = atan2(sqrt(1-cos_for_beta3**2), cos_for_beta3)
            beta4 = atan2(-a3, d4)
            theta3 = pi/2 - beta3 - beta4
            
            # make them to be numpy float64
            theta1, theta2, theta3 = np.float64(theta1), np.float64(theta2), np.float64(theta3)
            
            # Finding theta 4-6
            R0_3_T = R0_3_inv.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = np.array(R0_3_T * Rrpy).astype(np.float64)
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
                
            # make them to be numpy float64
            theta4, theta5, theta6 = np.float64(theta4), np.float64(theta5), np.float64(theta6)
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
