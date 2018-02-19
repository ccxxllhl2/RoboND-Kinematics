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
[image11]: ./misc_images/R36_detail.png
[image12]: ./misc_images/error.png
[image13]: ./misc_images/test01.png
[image14]: ./misc_images/test02.png
[image15]: ./misc_images/test03.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will describe each step I have done to achieve key points.

---
### Writeup / README

#### 1. First show my tools. I use Gazebo, RViz and moveit! to simulate the real physical environment of KR210.

You can see the picture of Gazebo as like below.
![alt text][image1]

### Kinematic Analysis
#### 1. Kinematic analysis always include two step, the Forward Kinematic and the Inverse Kinematic. to perform the Forward Kinematic analysis of Kuka KR210 robot I need to derive its DH parameters. In this section, I'll get how DH parameters assignment and how many they are. 

You can get some information here [FK and IK](http://www.cs.cmu.edu/~15464-s13/lectures/lecture6/IK.pdf), and here [Computation Detail](http://www.cs.columbia.edu/~allen/F15/NOTES/jacobians.pdf)

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

Use the rotation matrix and first 3 theta to calculate the last joint variables(theta4, theta5, theta6).  
* To do this, first transform the DH matrix to be this, and you should see the rotation part is just [0:3,0:3]
![alt text][image9]
* Then take out R0_3 from T0_3, to do this just get T0_3[0:3,0:3]
* Next transpose R0_3 and time with Rrpy(R0_6), that is the R3_6. 
![alt text][image10]
* The entire R3_6 is as below.
![alt text][image11]
* Get all six rotation parameters

param | value
  --- | ---
  r12 | R3_6[0, 1]
  r13 | R3_6[0, 2]
  r21 | R3_6[1, 0]
  r22 | R3_6[1, 1]
  r23 | R3_6[1, 2]
  r32 | R3_6[2, 1]
  r33 | R3_6[2, 2]

* Caculate theta5 as below  
$ \theta5 = atan2(sqrt(r13\*r13 + r33\*r13), r23) $

* Caculate theta4 and theta6 by pose of theta5, the detail will be talked in next section  
$ \theta4 = atan2(-r33, r13) $
$ \theta6 = atan2(r22, -r21) $

### Project Implementation

#### 1. White IK_server.py file such that syetem will take the Inverse Kinematic caculation process.  Then do optimization and play with the error.

The code will be departed in 3 section.  
* The first section is to set some function as like rot function for calculate Rotation Matrix with x,y and z axis. I make a new file "defined_var.py" to seperate some DH process. Also I define some variable to save DH parameters which will be used in IK, instead of dict "s". With "Matrix" of "Sympy" I can set a model, then feed in datas with "evalf(subs())"   
  
* The second section is the Forward Kinematic, that's mean I should set functions to get position(coordinates) by rotaton angle. In this section, I get DH parameters from UDRF file and set them to be symbols. To get help of the "defined_var.py", I set most caculation of Forward Kinematic out of the for loop in "IK_server.py".  
  
* The third section is the Inverse Kinematic, that's mean get angle based on position here.  
In this section, I use knowledge described up to get theta1-3 and calculate rotation matrix R0_3, here use "atan2" to get angle when sin and cos of this angle have been known.   
Then transpose R0_3 so that get R3_6. Caculation of R3_6 need the real-time value of theta1-3. So I turn R3_6 into numpy data after caculated. "R3_6 = np.array(R0_3_T * Rrpy).astype(np.float64)".   
Finally get last 3 theta, and make all of thetas into numpy(float64). Utilize IK_debug.py, I make the error to be not very high. It's very important to separate when "sin(theta5)>0" and "sin(theta5)<0". That's because there is different pose for the grab to reach the same position. Different pose will get different error. If we can define the pose of joint5, we'll know the pose of joint4.  

![alt text][image12]  

#### 2.Show the simulated environment and how my code worked.
Finaly my code get a not bad result, 9/10 pick and place.  

![alt text][image13]
![alt text][image14]

Even more, I found it could do other jobs :)

![alt text][image15]

#### 3.Problems
In my tests, two problems will cause failure of pick and place:
* grab fail: refer to class, we can modify /src/trajectory_sampler.cpp file, add "ros::Duration(2.0).sleep();", Advice is add more duration. Then do catkin_make.
* grab success but drop when arm moving: I have no idea now, but I think modify the limit range of grab's movement may be solve this.

#### 4.Extended work
I'll continue the job of this project. I think this [paper](https://arxiv.org/abs/1801.10425) is a good choice to refer.