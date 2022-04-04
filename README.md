---
typora-root-url: ./
---

# ur5_hand_eye_calibrate
基于easy_handeye开源项目，对ur5机械臂进行手眼标定（Kinect v2眼在手外）；里面主要是修改了一些easy_handeye中的一些launch文件，实现手眼转换矩阵的计算与发布。

## 依赖

1. Ubuntu18.04   ros-melodic

2. ur5机械臂，PolyScope固件版本3.9

3. 安装[kinect v2 ros相关包](https://github.com/code-iai/iai_kinect2.git)

4. 安装[easy_handeye](https://github.com/IFL-CAMP/easy_handeye)

5. 参照[官方教程](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)安装[Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) 与 [universal_robot](https://github.com/fmauch/universal_robot)包

6. 在`universal_robot/ur5_moveit_config/config`中创建`controllers.yaml`文件，并写入

   ```yaml
   controller_list:
     - name: ""
       action_ns: /scaled_pos_traj_controller/follow_joint_trajectory
       type: FollowJointTrajectory
       joints:
         - shoulder_pan_joint
         - shoulder_lift_joint
         - elbow_joint
         - wrist_1_joint
         - wrist_2_joint
         - wrist_3_joint
   ```

7. 按照[官方教程](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)，对ur5机械臂端进行配置，主要是在ur5端安装`externalcontrol-1.0.5.urcap` ，并设置工作站ip

8. 抽取机械臂校准参数

   ```bash
   roslaunch ur_calibration calibration_correction.launch \
     robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
   ```

   

## 安装

1. 安装本项目文件

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Hymwgk/ur5_hand_eye_calibrate.git
   cd ..
   catkin_make
   source ~/catkin_ws/devel/setup.bash
   ```
   
2. 完成包括依赖在内的所有安装后，确保src文件夹中存在如下结构：
    ```bash
    wgk@wgk:~/catkin_ws/src$ tree -L 1
    .
    ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
    ├── easy_handeye
    ├── fmauch_universal_robot
    ├── iai_kinect2
    ├── Universal_Robots_ROS_Driver
    └── ur5_hand_eye_calibrate
    
    5 directories, 1 file
    ```



## 标定

1. 自行安装好相机和标定板，标定板使用 [ar_track_alvar](http://wiki.ros.org/ar_track_alvar/)生成的10cm*10cm的二维码标签，id为7，确保相机能够观察到完整标签

   <img src="/README.assets/微信图片_20220404105908.jpg" alt="微信图片_20220404105908" style="zoom: 10%;" />

   

2. 单独启动底层controller，将ip地址更换为你的ur5机械臂固定ip，机械臂校准参数路径也设置为之前保存的路径

   ```bash
   roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.110 \
     kinematics_config:="${HOME}/my_robot_calibration.yaml"
   ```

3. 在ur5控制板上，启动与ros对接的程序，方法参见[此处](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)，注意，该操作必须在步骤2之后执行

<img src="/README.assets/微信图片_20220404115557.jpg" alt="微信图片_20220404115557" style="zoom:50%;" />

4. 启动顶层moveit相关节点

   ```bash
   roslaunch   ur5_hand_eye_calibrate   ur5_moveit_rviz.launch
   ```

5. 另一个命令窗口，打开相机,并进行监视，防止机械臂与与周边环境发生碰撞

   ```bash
   roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
   rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud
   ```

6. 打开窗口进行监视

   ```bash
   rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud
   ```

7. 另一个命令窗口，进行标定，按照提示进行标定

   ```bash
   roslaunch ur5_hand_eye_calibrate ur5_eob.launch
   ```

## 已标定&使用

1. 发布手眼姿态矩阵

   ```bash
   roslaunch ur5_hand_eye_calibrate publish_ur5_eob.launch
   ```
   
2. 解锁ur5机械臂，并启动Moveit

   ```bash
   roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.110   kinematics_config:="${HOME}/my_robot_calibration.yaml"
   #另一窗口
   roslaunch   ur5_hand_eye_calibrate   ur5_moveit_rviz.launch
   ```

## TroubleShooting

   如果tf出现问题，就先重新启动Moveit步骤

<img src="/README.assets/image-20220403221438766.png" alt="image-20220403221438766" style="zoom:50%;" />

