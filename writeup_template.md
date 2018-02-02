## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/KR210_pic.png
[image3]: ./misc_images/urdf_file.png
[image4]: ./misc_images/DH_parm.png
[image5]: ./misc_images/WC.png
[image6]: ./misc_images/theta1.png
[image7]: ./misc_images/theta2.png
[image8]: ./misc_images/theta3.png
[image9]: ./misc_images/rr.png
[image10]: ./misc_images/R36.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading the writeup file about the project 2 of Udacity's Robotic ND.
![alt text][image1]

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I derived the DH Params from the below axis assignments. I found the location of a3 in the videos of class maybe wrong, because a3 is measured along x3, that's mean a3||x3. I think it's just the different way to reference the frame.

![alt text][image2]

And I get these DH parameters from kr210.urdf.xacro file, as like this.

![alt text][image3]

The link state of joints are descripted here.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Then I use these DH parameters in IK\_server.py.

![alt text][image4]

For solving the problem of low-speed of IK process, I set rot and homo_tf the two functions which modify some calculation of Sympy module out of the for loop.
And also I rewrite some DH param for IK process, these param will be used twice every for loop. I don't know if doing this will make any problem in IK process, but it's very useful to reduce the calculating time. 
* a1 = 0.35
* a2 = 1.25
* a3 = -0.054
* d1 = 0.75
* d4 = 1.5
* d7 = 0.303

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

As DH way to describe and solve the Inverse Kinematics problem. First step is to choice the wrist center(WC), and find the xyz relation from the base frame. Take joint 5 to be the WC

![alt text][image5]

$ Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr $  
Which Rot is the rot function.

Then use the position of WC and end effector (EE) to calculate the first 3 joint variables(theta1, theta2, theta3), and the math is described in the below pic.

![alt text][image6]
![alt text][image7]
![alt text][image8]

Next use the rotation matrix to calculate the last joint variables(theta4, theta5, theta6). To do this, first transform the DH matrix to be this.
![alt text][image9]
![alt text][image10]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

In the IK\_server.py, T0_3 is the only T parameter we need to get R0_3 in the IK for loop, so I remove others. Then use inv R0_3 to get R0_6.
Also the arm will not work so currectly because of the error. I'll do more test about the key joint theta5.