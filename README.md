# Project-2

## Instruction
In waste sorting, 6D picking is one of the most effective ways to pick and categorize litters. In this assignment, we carried out this process including calibration and picking. We first conducted the 3D calibration. We ran the program provided by the course to calculate the homogeneous transformation matrix between the robot base frame and the camera frame using information obtained from the image and feedback from the robot arm. Then we applied the result of calibration and controlled the robot arm to conduct a series of motions：picking up a bottle, taking it to traverse a certain path, and releasing the bottle. The bottle is first detected by the camera. Its position information under the camera frame is obtained and transformed to robot base frame using the transformation matrix obtained in calibration. The expected joint state of the robot arm at each time frame is calculated using inverse kinematics and input into each actuator. Actuators control the robot arm to move and complete the expected tasks. Thanks to this assignment, we learn how to conduct 6D picking using the robot arm.

## Equipment
The manipulator we used in the experiment is Franka Emika Panda, which is a 7-DoF robotic arm. Each DoF is actuated by a brushless dc motor. The end effector can be selected between the flange and the Franka hand. In this assignment, we select the Franka hand as the end effector. A gantry made of European standard 4040 aluminum profile is installed in front of the robot arm, and an Intel Realsense D435i camera is installed in the middle of the aluminum gantry and facing the conveyor belt. A MSI Trident 3 computer is used to run the program controlling the robotic arm to move.

## Experiment Procedure

### 3D Calibration
#### Principle of 3D Calibration
The goal of 3D hand-eye calibration is to obtain a hand-eye transformation matrix, which is used to describe the relative spatial pose between the robotic arm and the camera, so as to convert a certain point p (x, y, z) in the camera coordinate system into a mechanical point p'(x', y', z') under the arm base coordinates.

![Principle_of_3D_Calibration_1](https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Principle_of_3D_Calibration_1.png)
![Principle_of_3D_Calibration_2](https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Principle_of_3D_Calibration_2.png)

In the process, we collected multiple sets of images of the calibration board and read the position of the board center in the camera frame $^{Cam}_{Marker}p$. Also, we read the position and posture of TCP(Tool central point) from the feedback of the robotic arm and use them to calculate the homogenous transformation matrix from the base to TCP $$^{Base}_{TCP}H$$. Plus, the position of the center of the calibration board $$^{TCP}_{Marker}p$$can be measured. Thus,$$^{Base}_{Cam}H$$ can be solved. In this assignment, the numerical SVD method is applied to obtain the optimal solution of $$^{Base}_{Cam}H$$.

The equation to be solved is in the form $$Xa=b$$, where $$X$$ is an unknown homogeneous transformation matrix to be solved. Since $$X$$ describes a transformation which can be uniquely determined by 6 independent unknown variables (x, y, z, yaw, pitch, roll), and using one calibration point a set of three linear equations can be generated:
 $$\begin{bmatrix}x_1^{T} \\ x_2^{T}\\x_3^{T}\\c\end{bmatrix}a=b, $$where $$c = [0\ 0\ 0\ 1]$$
$$\Rightarrow \left\{\begin{aligned}x_1^{T}a=b_1\quad(1) \\ x_2^{T}a=b_2 \quad (2)\\ x_3^{T}a=b_3 \quad (3)\end{aligned} \right.$$
So theoretically only two calibration points are needed to solve $$^{Base}_{Cam}H$$. However, to make the calibration results more accurate, we have the robotic arm walk a 4×4 grid so that the calibration error can be averaged. 


#### Principle of 3D Calibration
1. Start the Franka robotic arm and adjust its the end to a moderate position.
2. Install the calibration board and make it face the camera.
3. Open the cail3D.yaml file and modify the following parameters: the initial posture of the robot arm, the grid step length, the offset of the calibration plate relative to the end effector of the robotic arm.
4.  Run EyeOnBase.py, and the robotic arm will start to walk the grid. Wait for completion of the calibration. The calibration process will generate the acquisition data file *.npz and the hand-eye matrix file *.npy.
5. Select four points on the conveyor belt in the 3D view, and use plane_calculate.py to calculate the filter plane. Finally, copy the parameters of the filter plane and paste them in the proper place in main.py.

### 6D Picking

#### Initialization
Before picking, the prpgram needs to obtain some information
1. Controler parameters.(velocity, acceleration, home joint and so on).
2. Transform relationship between camera image and fraka position based on the calibration result. 
3. Trained Yolo5 network
4. Camera parameters
After obtaining this parameters, the franka will move to the home points. And we define the moving area limit of the end effector.
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
After obtaining the position of garbage in the image, the program will calculate the position of bottle in the camera frame according to the pinhole imaging principle. The bottle position is represented by the two endpoints of the diagonal of a rectangular enveloping which almost exactly contains the bottle. 
In the program, we use a plane filter to remove points which is below the target plane.
```
for i in range(len(pc)):
            if pc[i][0] * plane_model[0] + pc[i][1] * plane_model[1] + pc[i][2] * plane_model[2] + plane_model[3] < 0:
                new_pc.append(pc[i])
```


#### Transformation
The program will calculate the pose of the project based on its envelope in the camera frame. With the result of 3D calibration, the program will convert the position and orientation of the bottle from the camera frame to the robotic base frame. In this part, the program uses a function in the `deepclaw/modules/grasp_planning/GeoGrasp` part.
```
from deepclaw.modules.grasp_planning.GeoGrasp import GeoGrasp
m = GeoGrasp.run(new_pc)
grasp_matrix = np.reshape(m, (4, 4), order='C')
grasping_in_base = np.dot(hand_eye_matrix, grasp_matrix)
```

#### Moving
After obtaining the position and orientation of the bottle in the robot base frame, the program uses the inverse kinematics through the underlying drive to obtain the expected angles of each joint. The program uses an E_T_F matrix to calculate the target orientation of the grasp and the pose when the grasp has raised. 
```
E_T_F = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.133], [0, 0, 0, 1]])
# grasping_in_base = np.dot(grasping_in_base, E_T_F)
grasping_in_base = np.dot(grasping_in_base, np.linalg.inv(E_T_F))
r = R.from_matrix(grasping_in_base[0: 3, 0: 3])
rot = r.as_euler('xyz', degrees=False)
transfer = grasping_in_base[0:3, 3]
measure_z = -0.13  # meter
offset_y = 0.01
temp_pose = [transfer[0], transfer[1]+offset_y, transfer[2] + measure_z, rot[0], rot[1], rot[2]]

# z offset ,  pick up
E_T_F = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.03], [0, 0, 0, 1]])
grasping_in_base = np.dot(grasping_in_base, E_T_F)
r = R.from_matrix(grasping_in_base[0: 3, 0: 3])
rot = r.as_euler('xyz', degrees=False)
transfer = grasping_in_base[0:3, 3]
temp_pose_up = [transfer[0], transfer[1], transfer[2], rot[0], rot[1], rot[2]]

pick_place(robot, robot, home_joints, temp_pose, temp_pose_up, place_xyzrt)
```


## Demo Video
- [Test Video1](https://bionicdl.feishu.cn/file/boxcnnREvnRSeLRXvhLtISQo6Zf?from=from_copylink)
- [Test Video2](https://bionicdl.feishu.cn/file/boxcnrVm5QLV5pRB1ziDfSMI4yc)

<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/demo.GIF" width="50%">

## Problems and Solutions

### Problem: The end-point problem between using Franka hand or not.
After doing 3D eye to hand calibration, we run the 6D picking code and find the position and posture we want has a constant off-set between the real word robot arm position. After several tests, we find the problem. When we do 3D clibration, we choose 'none end effctor'. At this time, the end point of robot arm is the frange at the end of robot arm. However, when we do 6D picking, we choose 'Franka hand' as end effector, the end point and the frame will change into Tool Center Point（TCP). Finally, we decide to install the fiducial marker and franka hand together and change the End_to_Flange matrix. 

<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Problem_Solutions_1.png" width="50%"><img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Problem_Solutions_2.png" width="50%">

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
#### 1. Correcting the End_To_Flange matrix according to our installation
We find there are some problems in the off-set setting, because we fix the calibration board in a different way from the picture in tutorial. The off-set parameter in E_T_F(end point to flange) matrix should be changed according to our practical measurement in our calibration process.

<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Off-set_setting_problem_1.png" width="50%"><img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Off-set_setting_probelm_2.png" width="50%">

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
  
#### 2. Verifying that the calibration results are correct
Due to the large error of picking, we need to find where the trouble is. 

First, we want to find out how large the calibration error is. So we use a big calibration board which can test how accurate the transformation matrix from the base to the camera $$^{Base}_{Cam}H$$. The tool in opencv can help us find the origin of the big calibration board coordinate (The tip of the arrow on the right side in the following picture). Using $$^{Base}_{Cam}H$$ obtained in the calibration and the position of the origin of the big calibration board read from the camera $$^{Cam}_{Origin}p$$, we can calculate the position of the origin of the calibration board $$^{Base}_{Origin}p$$( $$^{Base}_{Origin}p$$= $$^{Base}_{Cam}H$$ $$^{Cam}_{Origin}p$$). To verify that the calculated$$^{Base}_{Origin}p$$is correct (i.e. Relatively accurate), so that $$^{Base}_{Cam}H$$ obtained in calibration is correct, we control the robotic arm move to this calculated position. By measuring the distance between the TCP of the robot arm and the origin of the big calibration board, we can see that the error is very small (Indicated by the two arrows on the left side in the following picture). This means that our eye to hand calibration result (i.e.  $$^{Base}_{Cam}H$$ obtained in calibration) is correct. 

<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Test_of_3D_calibration_result_1.png" width="50%">
 
Note that the end effector of the robotic arm may not move to the expected position or posture. For example, in the following picture, the desired position and posture ([0.5047, -0.0179, 0.2, 1.57, -1.57, 1.57]) do not match the actual one([0.4347, -0.0239, 0.1346, 3.0524, -0.3824, -0.0512]). Hence, we need to check whether or not the end effector gets the desired position and posture using the function robot.get_state() after we control it to move to the calculated position and posture of the big calibration board.

<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Test_of_3D_calibration_result_2.png" width="50%">
  
#### 3. Verifying that the bottle's object detection result is correct
We hope to find out whether the problem is in the YOLOV5 object detection process, so we want to figure out how accurate the detection result is. We add a line of code to print out the objection detection result.
```
def detectObject(detector_algo: Yolo5, color, crop_bounding=[300, 720, 300, 1000]):
  . . .
  . . .
    # pick one object once
    uv = uv_roi[0]
    cla = cla_roi[0]
    cfi = cfi_roi[0]
    
    print(f'the bolttle uv is {uv}')           # 加入此行，打印出检测的位置
    
    return True, uv, cla, cfi
```
<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Test_of_the_bottle's_object_dection_1.png">

The detection result is [481, 311, 543, 475], which should be the uv values of the two corner points of the bottle. (In the form of [u1, v1, u2, v2]). Then we open the realsense-viewer and find whether the detection result is indeed the uv values of the two corner points. 

<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Test_of_the_bottle's_object_dection_2.png" width="50%"><img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Test_of_the_bottle's_object_dection_3.png" width="50%">

The uv values of bottle in realsense-viewer is [487,315,541,475]. This is very close to the result of YOLOV5's detection results. So the detection result is correct.
  
#### 4. Finding that the intrinsic parameters of the camera are wrong in the program provided by the course
In the program provided by the course, the x and y coordinates of the point cloud of the bottle in the base frame are calculated according to the YOLO5's detection result (uv values of the corner two points):
```
for i in range(uv[1], uv[3]):
            temp_pc = []
            for j in range(uv[0], uv[2]):
                z = depth_img[i][j]
                x = (j - cx) * z / fx   #x coordinate of point cloud
                y = (i - cy) * z / fy   #y coordinate of point cloud
```
where fx, fy, cx, and cy are intrinsic parameters of the camera.  We did a test to find whether there are problems with the calculated point cloud positions. We compare the position of the plotted point cloud and that of the bottle shown in the camera, as is shown in the following two pictures.

<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/The_problem_of_camera's_intrinsic_parameter_1.png" width="50%"><img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/The_problem_of_camera's_intrinsic_parameter_2.png" width="50%">
We find that the two do not match each other (The x coordinate of the plotted point cloud center is about -0.26, while that of the bottle center in the camera is -0.157).  Since the detection result is correct, we think there may be problems with the intrinsic parameters. It seems that this cannot be true, since the 3D calibration process also involves the camera's intrinsic parameters while its results are correct. However, we find that in calibration, the intrinsic parameters are got from Realsense-SDK automatically (line 4 - line 7):
```
def image_callback(color_image, depth_image, intrinsics):
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    fx = intrinsics[0]
    fy = intrinsics[1]
    cx = intrinsics[2]
    cy = intrinsics[3]
    print(f'The intrinsic paras are {inrtinsics}')
```
But when calculating the position of the point clouds, they are written manually (line 5 - line 8):
```
 # transfer to point cloud
        # depth intrinsics
        width = 1280
        hight = 720               #here are intrinsic parameters for D435  
        fx = 640.983              # wrong intrinsic parameters
        fy = 640.983   
        cx = 641.114
        cy = 368.461  
        pc = []
```
So it is reasonable that the calibration result is correct, while the calculated position of the point clouds is incorrect due to wrong written intrinsic parameters.
There are two methods to find out the correct intrinsic parameters:
1. Directly printing out the intrinsic parameters used for calibration:
<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/The_problem_of_camera's_intrinsic_parameter_3.png">
2. Getting intrinsic parameters using commands in the cmd:
<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/The_problem_of_camera's_intrinsic_parameter_4.png">
It is shown that the written intrinsic parameters are indeed wrong. This is because our camera's model, which is D415, is different from other teams, which is D435, and their intrinsic parameters are different. The program provided by the course is for D435.
<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/The_problem_of_camera's_intrinsic_parameter_5.png">
After correcting the written intrinsic parameters used for calculation of the point cloud, we finally got the result of picking correct.


## Possible Improvements
There are several possible improvements.
1. We may create an environment with a more stable light condition for manipulation. The whole system is placed beside the window, so the light condition is significantly influenced by the sunlight, which may have an impact on vision recognition. Thus, we may draw the curtain and apply a stable light condition.
2. We may replace the Franka hand with the robotic hand designed by ourselves. This is because the Franka hand does not work well. Its two fingers sometimes cannot clamp when grasping the bottles due to poor connection with its controller. On top of that,the Franka hand might make the bottles deform while grasping because the tips of the hand are rigid. If we have time, we are going to design an end effector with flexible tips.
3. We may fix our conveyor belt, robotic arm and aluminum gantry together using some structures. This is because they easily move relative to each other with external disturbance like accidental collision or even touch. Calibration needs to be performed once again after this, which causes much trouble for us. The following picture shows a connection structure devised by us. We integrate the camera frame and conveyor belt together and use section bar to limit the location of Franka. With connector, base station can be fixed to conveyer belt and easy to apart. It avoids the relative displacement among camera, Franka and working plantform. Ideally, with this structure, there is no need to calibrate every time before working.
<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Possible_Improvements_1.png" width="50%">

4. We may apply new methods generating a point cloud which can more accurately depict the shape of the bottle . As is shown in the following figure, the point cloud contains a lot of noise, thus only roughly depicting the shape of the bottle. Therefore, the direction of the end effector approaching the bottle tends to be arbitrary. Its attitude can change significantly even if the position and attitude of the bottle remains the same. This brings us problems:  In some directions, the hand cannot pick up the bottle. If the point cloud is an accurate depiction, the robot arm can effectively decide the most proper direction approaching the item to be grasped according its shape. One possible method to improve the quality of the point cloud is filtering the noise.
<img src="https://github.com/Sustech2021-ME336-Team-Green/Project-2/blob/main/images/Possible_Improvements_2.png" width="50%">
5. We may make the connection between the camera and the computer more stable. The connection between these two sometimes goes wrong, and we have to plug out the wire and plus it in again.


## Contributions
- System Engineer: Nuofan Qiu
- Algorithm Engineer: Xin Liu, Yifei Li
- Software Engineer: Ronghan Xv, Yujian Dong 
- Data Engineer: Yanzhen Xiang, Shangkun Guo
- Design Engineer: Yang Xiao, Bowen Hu

