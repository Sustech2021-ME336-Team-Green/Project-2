# Project-2

## Instruction
In waste sorting, 6D picking is one of the most effective ways to pick and categorize litters. In this assignment, we carried out this process including calibration and picking. We first conducted the 3D calibration. We ran the program provided by the course to calculate the homogeneous transformation matrix between the robot base frame and the camera frame using information obtained from the image and feedback from the robot arm. Then we applied the result of calibration and controlled the robot arm to conduct a series of motions：picking up a bottle, taking it to traverse a certain path, and releasing the bottle. The bottle is first detected by the camera. Its position information under the camera frame is obtained and transformed to robot base frame using the transformation matrix obtained in calibration. The expected joint state of the robot arm at each time frame is calculated using inverse kinematics and input into each actuator. Actuators control the robot arm to move and complete the expected tasks. Thanks to this assignment, we learn how to conduct 6D picking using the robot arm.

## Equipment
The manipulator we used in the experiment is Franka Emika Panda, which is a 7-DoF robotic arm. Each DoF is actuated by a brushless dc motor. The end effector can be selected between the flange and the Franka hand. In this assignment, we select the Franka hand as the end effector. A gantry made of European standard 4040 aluminum profile is installed in front of the robot arm, and an Intel Realsense D435i camera is installed in the middle of the aluminum gantry and facing the conveyor belt. A MSI Trident 3 computer is used to run the program controlling the robotic arm to move.

## Experiment Procedure

### 3D Calibration
The goal of 3D hand-eye calibration is to obtain a hand-eye transformation matrix, which is used to describe the relative spatial pose between the robotic arm and the camera, so as to convert a certain point p (x, y, z) in the camera coordinate system into a mechanical point p'(x', y', z') under the arm base coordinates.



#### Principle of 3D Calibration
1. Start the Franka robotic arm and adjust the end position of the robotic arm.
2. Fix the calibration board and make it face the camera.
3. Open the cail3D.yaml file and modify the following parameters: the initial posture of the robot arm, the grid step length, the offset of the calibration plate relative to the end flange of the robot arm
4. Run EyeOnBase.py, the robotic arm will start to walk the grid. Wait for the end of the robot arm movement to complete the calibration. So we can get the calibration acquisition data file *.npz and the hand-eye matrix *.npy.
5. Select four points on the conveyor belt in the 3D view, use plane_calculate.py to calculate the filter plane, and finally use the parameters of the filter plane in the main program

### 6D Picking

#### Initialization
Before picking, the prpgram needs to obtain some information
1. Controler parameters.(velocity, acceleration, home joint and so on).
2. Transform relationship between camera image and fraka position based on the calibration result. 
3. Trained Yolo5 network
4. Camera parameters
After obtaining this parameters, the franka will move to the home points. And we define the max moving area of Franka.
```
""" Initialization """
# camera and robot driver
print('work_dir: ', _root_path)
robot = FrankaController('./configs/basic_config/franka.yaml')
camera = Realsense('./configs/basic_config/camera_rs_d435.yaml')
object_detector = Yolo5('./configs/basic_config/yolov5_cfg.yaml')
hand_eye_matrix = np.load('./configs/E_T_B.npy')

home_joints = [-0.03, -1.3, 0.05, -2.2, 0.08, 1.15, 0.7]
robot.move_j(home_joints, 1.5, 1.5)    
place_xyzrt = [0.3, -0.5, 0.4, 3.14, 0.0, -1.57]
crop_bounding = [250, 500, 320, 1000]
```


#### Detection
In this part, we obtain the color image from Realsence and use trained YOLO5 model to recognize the trash in the crop_bouding of the image.
```
frame = camera.get_frame()
color = frame.color_image[0]
depth_img = frame.depth_image[0]
# region_class = object_detector.detect(color)
ret, uv, cla, cfi = detectObject(object_detector, color, crop_bounding=[250, 500, 320, 1000])
```
After obtaining the position of garbage in the image, the program will calculate the position of garbage relative to the camera coordinate system according to the pinhole imaging principle.



#### Transformation

#### Moving


## Demo Video
[Test Video](https://bionicdl.feishu.cn/file/boxcnrVm5QLV5pRB1ziDfSMI4yc)

## Problems and Solutions

### Problem: The end-point problem between using Franka hand or not.
After doing 3D eye to hand calibration, we run the 6D picking code and find the position and posture we want has a constant off-set between the real word robot arm position. After several tests, we find the problem. When we do 3D clibration, we choose 'none end effctor'. At this time, the end point of robot arm is the frange at the end of robot arm. However, when we do 6D picking, we choose 'Franka hand' as end effector, the end point and the frame will change into Tool Center Point（TCP). Finally, we decide to install the fiducial marker and franka hand together and change the End_to_Flange matrix. 

### Problem:The picking error is large after 3D calibration 
To begin with, our 3D calibration's performance is not good enough. Here is our testing data.

|       | Position x(camera frame) | Position y(camera frame) | Error x(base frame) | Error y(base frame) |
| ----  | ------------------------ | ------------------------ | ------------------- | ------------------- |
| Test1 | 347pix                   | 387pix                   | -4cm                | -17.5cm             |
| Test2 | 534pix                   | 385pix                   | -4cm                | -2cm                |
| Test3 | 652pix                   | 393pix                   | -4cm                | 0.5cm               |
| Test4 | 810pix                   | 406pix                   | -3cm                | 4cm                 |
| Test5 | 1004pix                  | 406pix                   | -3cm                | 17cm                |

After testing we find the error of x,y is lager when the bottle is far away from calibration initial point. After seeking help from Dr.Wan, and know it is a system error when doing 3D calibration. The calibration error will be large when the work space is far away from the calibration place.

### Solution: Trouble shoot of picking error
#### Off-set setting problem
We find there are some problems in the off-set setting, because we fix the calibration board in a different way from the picture in tutorial. The off-set parameter in E_T_F(end point to flange) matrix should be changed according to our practical measurement in our calibration process.

The `cali3D.yaml` file in tutorial.
```
E_T_F:
  - [1,0,0,0.06]
  - [0,1,0,0.00]
  - [0,0,1,0.03]
  - [0,0,0,1]
OFFSET:
  x: 0.06
  y: 0.0
  z: 0.03
  ```
  The correct off-set data.
  ```
  E_T_F:
  - [1,0,0,0.06]
  - [0,1,0,0.00]      
  - [0,0,1,-0.1334]
  - [0,0,0,1]
OFFSET:
  x: 0.06
  y: 0.0
  z: -0.09
  ```
  
#### Test of 3D calibration result
  
#### Test of the bottle's object dection
  
#### The problem of camera's intrinsic parameter


## Possible Improvements
There are several possible improvements.
1. We may create an environment with a more stable light condition for manipulation. The whole system is placed beside the window, so the light condition is significantly influenced by the sunlight, which may have an impact on vision recognition. Thus, we may draw the curtain and apply a stable light condition.
2. We may replace the Franka hand with the robotic hand designed by ourselves. This is because the Franka hand does not work well. Its two fingers sometimes cannot clamp when grasping the bottles due to poor connection with its controller. On top of that,the Franka hand might make the bottles deform while grasping because the tips of the hand are rigid. If we have time, we are going to design an end effector with flexible tips.
3. We may fix our conveyor belt, robotic arm and aluminum gantry together using some structures. This is because they easily move relative to each other with external disturbance like accidental collision or even touch. Calibration needs to be performed once again after this, which causes much trouble for us. The following picture shows a connection structure devised by us. We integrate the camera frame and conveyor belt together and use section bar to limit the location of Franka. With connector, base station can be fixed to conveyer belt and easy to apart. It avoids the relative displacement among camera, Franka and working plantform. Ideally, with this structure, there is no need to calibrate every time before working.
4. We may apply new methods generating a point cloud which can more accurately depict the shape of the bottle . As is shown in the following figure, the point cloud contains a lot of noise, thus only roughly depicting the shape of the bottle. Therefore, the direction of the end effector approaching the bottle tends to be arbitrary. Its attitude can change significantly even if the position and attitude of the bottle remains the same. This brings us problems:  In some directions, the hand cannot pick up the bottle. If the point cloud is an accurate depiction, the robot arm can effectively decide the most proper direction approaching the item to be grasped according its shape. One possible method to improve the quality of the point cloud is filtering the noise.
5. We may make the connection between the camera and the computer more stable. The connection between these two sometimes goes wrong, and we have to plug out the wire and plus it in again.


## Contributions
- System Engineer: Nuofan Qiu
- Algorithm Engineer: Xin Liu, Yifei Li
- Software Engineer: Ronghan Xv, Yujian Dong 
- Data Engineer: Yanzhen Xiang, Shangkun Guo
- Design Engineer: Yang Xiao, Bowen Hu

