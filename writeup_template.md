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
[image11]: ./misc_images/error.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will describe each step I have done to achieve key points.

---
### Writeup / README

#### 1. First show my tools. I use Gazebo, RViz and moveit! to simulate the real physical environment of KR210.

You can see the picture of Gazebo as like below.
![alt text][image1]

### Kinematic Analysis
#### 1. Kinematic analysis always include two step, the Forward Kinematic and the Inverse Kinematic. to perform the Forward Kinematic analysis of Kuka KR210 robot I need to derive its DH parameters. In this section, I'll get how DH parameters assignment and how many they are. 

I derived the DH parameters from the below axis assignments in the pic I've drew.
![alt text][image2]

And I get these DH parameters from kr210.urdf.xacro file. You find this fine in \kuka_arm\urdf as below.

![alt text][image3]

The DH parameters of every joints are descripted here.  
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | 0
1->2 | - pi/2 | 0.35 | 0 | q2-pi/2
2->3 | 0 | 1.25 | 0 | 0
3->4 | - pi/2 | -0.054 | 1.5 | 0
4->5 | pi/2 | 0 | 0 | 0
5->6 | - pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

#### 2. In this section, I'use the DH parameter table I derived earlier, create individual transformation matrices about each joint. In addition,  I also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The transformation matrices for joint 1->2

--- | --- | --- | ---
--- | --- | --- | ---
sin(q2) |  cos(q2) |  0 |  0.350000000000000 
0 |  0 |  1 |  0 
cos(q2) |  -sin(q2) |  0 |  0 
0 |  0 |  0 |  1
The transformation matrices for joint 2->3

--- | --- | --- | ---
--- | --- | --- | ---
cos(q3) |  -sin(q3) |  0 |  1.25000000000000 
sin(q3) |  cos(q3) |  0 |  0 
0 |  0 |  1 |  0 
0 |  0 |  0 |  1
The transformation matrices for joint 3->4

--- | --- | --- | ---
--- | --- | --- | ---
cos(q4) |  -sin(q4) |  0 |  -0.0540000000000000 
0 |  0 |  1 |  1.50000000000000 
-sin(q4) |  -cos(q4) |  0 |  0 
0 |  0 |  0 |  1
The transformation matrices for joint 4->5

--- | --- | --- | ---
--- | --- | --- | ---
cos(q5) |  -sin(q5) |  0 |  0 
0 |  0 |  -1 |  0 
sin(q5) |  cos(q5) |  0 |  0 
0 |  0 |  0 |  1
The transformation matrices for joint 5->6

--- | --- | --- | ---
--- | --- | --- | ---
cos(q6) |  -sin(q6) |  0 |  0 
0 |  0 |  1 |  0 
-sin(q6) |  -cos(q6) |  0 |  0 
0 |  0 |  0 |  1
The transformation matrices for joint 6->G

--- | --- | --- | ---
--- | --- | --- | ---
1 |  0 |  0 |  0 
0 |  1 |  0 |  0 
0 |  0 |  1 |  0.303000000000000 
0 |  0 |  0 |  1
The homogeneous transform between base_link and gripper_link(T0_G with all peremeters 'q' be 0)

--- | --- | --- | ---
--- | --- | --- | ---
-1.00000000000000 |  0 |  0 |  2.15300000000000 
0 |  1.00000000000000 |  0 |  0 
0 |  0 |  -1.00000000000000 |  1.94600000000000 
0 |  0 |  0 |  1.00000000000000
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

As DH way to describe and solve the Inverse Kinematics problem. First step is to choice the wrist center(WC), and find the xyz relation from the base frame. Take joint 5 to be the WC

![alt text][image5]

$ Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr $  
Which Rot is the rot function.

Then use the position of WC and end effector (EE) to calculate the first 3 joint variables(theta1, theta2, theta3), and the math is described in the below pic.

![alt text][image6]
![alt text][image7]
![alt text][image8]

Next use the rotation matrix and first 3 theta to calculate the last joint variables(theta4, theta5, theta6). To do this, first transform the DH matrix to be this.
![alt text][image9]
![alt text][image10]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code will be departed in 3 section.  
* The first section is to set some function as like rot function for calculate Rotation Matrix with x,y and z axis, and also there is a function to caculate the Homogeneous Transforms Matrix.  
* The second section is the Forward Kinematic, that's mean is I should set functions to get position(coordinates) by rotaton angle. In this section, I get DH parameters from UDRF file and set them to be symbols. T0_3 is a usable variable of next section so I calculate it here.
* The third section is the Inverse Kinematic, that's mean get angle based on position here. In this section, I use knowledge described up to get theta1-3 and calculate rotation matrix R0_3, then Inv R0_3 get R3_6. Finally get all 6 theta. Utilize IK_debug.py, I make the error to be not very high. It's very important to separate when theta5>0 and theta5<0. Different pose will get same position but different error. If we can define the pose of joint5, we'll know the pose of joint4.
![alt text][image11]