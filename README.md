# Project-2

## Instruction
In waste sorting, 6D picking is one of the most effective ways to pick and categorize litters. In this assignment, we carried out this process including calibration and picking. We first conducted the 3D calibration. We ran the program provided by the course to calculate the homogeneous transformation matrix between the robot base frame and the camera frame using information obtained from the image and feedback from the robot arm. Then we applied the result of calibration and controlled the robot arm to conduct a series of motions：picking up a bottle, taking it to traverse a certain path, and releasing the bottle. The bottle is first detected by the camera. Its position information under the camera frame is obtained and transformed to robot base frame using the transformation matrix obtained in calibration. The expected joint state of the robot arm at each time frame is calculated using inverse kinematics and input into each actuator. Actuators control the robot arm to move and complete the expected tasks. Thanks to this assignment, we learn how to conduct 6D picking using the robot arm.

## Equipment
The manipulator we used in the experiment is Franka Emika Panda, which is a 7-DoF robotic arm. Each DoF is actuated by a brushless dc motor. The end effector can be selected between the flange and the Franka hand. In this assignment, we select the Franka hand as the end effector. A gantry made of European standard 4040 aluminum profile is installed in front of the robot arm, and an Intel Realsense D435i camera is installed in the middle of the aluminum gantry and facing the conveyor belt. A MSI Trident 3 computer is used to run the program controlling the robotic arm to move.

## Experiment Procedure

### 3D Calibration

#### Principle of 3D Calibration

## Demo Video

## Problems and Solutions

## Possible Improvements

## Contributions

