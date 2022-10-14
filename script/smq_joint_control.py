#!/usr/bin/env python
#coding=utf-8
from cmath import pi
import math
from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
import numpy as np
import  xlrd
import rospy
import time
from std_msgs.msg import Float64MultiArray

from controller_manager_msgs.srv import SwitchController,SwitchControllerRequest,SwitchControllerResponse
from threading import Lock, Event

#注意，使用rostopic echo  /joint_states   显示的默认顺序为['elbow_joint','shoulder_lift_joint','shoulder_pan_joint',  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
JOINT_NAMES = ['elbow_joint','shoulder_lift_joint','shoulder_pan_joint',  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']




class SMQ():
    def __init__(self,data_path,init_angle,step,sim=False):
        #初始化ros节点
        rospy.init_node("pub_action_test")
        self.lock=Lock()
        self.switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.vel_pub=rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)

        #实例化一个action的类，命名为client，与上述client对应，话题为arm_controller/follow_joint_trajectory，消息类型为FollowJointTrajectoryAction
        if sim:
            self.position_control_client = actionlib.SimpleActionClient('/pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        else:
            self.position_control_client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            #保证初始是位置控制
            with self.lock:
                srv = SwitchControllerRequest()
                #con.start_controllers = 'position_joint_trajectory_controller'
                srv.strictness= srv.STRICT
                srv.stop_controllers =['joint_group_vel_controller']
                srv.start_controllers = ['scaled_pos_joint_traj_controller']
                self.switcher(srv)




        print("Waiting for server...")
        #等待server
        self.position_control_client.wait_for_server()
        print("Connect to server")
        self.waypoints = self.read_xls(data_path)
        self.init_angle = init_angle
        self.step =step
        self.rate=rospy.Rate(10)
        #执行move函数，发布action
        if sim:# 位置控制
            self.smq_move_position()
        else:#速度控制
            self.smq_move_vel()

        
    def smq_move_vel(self):
    #首先位置控制返回到初始位置
        joint_states = rospy.wait_for_message("joint_states",JointState)
        joints_pos = joint_states.position
        #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
        goal = FollowJointTrajectoryGoal()
        #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
        goal.trajectory = JointTrajectory()
        #goal.trajectory底下一共还有两个成员，分别是joint_names和points，先给joint_names赋值
        goal.trajectory.joint_names = joint_states.name

        #print("Joint Names: {}".format(joint_states.name))
        #print(" Joints: {}".format(joints_pos))

        #第一点需要是当前位置点
        goal.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(0.0)))
        #第二点设定回到初始位置，
        init_pose = list(joints_pos)
        #theta1    'shoulder_lift_joint'
        init_pose[1] = -self.init_angle[0]
        #theta2    'elbow_joint'
        init_pose[0] = self.init_angle[0]-self.init_angle[1]
        #theta3    'wrist_1_joint'
        init_pose[3] =  self.init_angle[1]-self.init_angle[2]-pi/2
        #
        init_pose[2] = 0.0
        init_pose[4] = 0.0
        init_pose[5] = 0.0

        init_time = 10.0
        goal.trajectory.points.append(JointTrajectoryPoint(positions=init_pose, velocities=[0]*6,time_from_start=rospy.Duration(init_time)))
        self.position_control_client.send_goal(goal)
        self.position_control_client.wait_for_result()
        
        time.sleep(2)
        joint_states = rospy.wait_for_message("joint_states",JointState)
        joints_pos = joint_states.position
        print("Current Joints: {}".format(joints_pos))

        
    #切换控制器，切换到速度控制器
        with self.lock:
            srv = SwitchControllerRequest()
            #con.start_controllers = 'position_joint_trajectory_controller'
            srv.strictness= srv.STRICT
            srv.start_controllers =['joint_group_vel_controller']
            srv.stop_controllers = ['scaled_pos_joint_traj_controller']
            self.switcher(srv)
            
        #速度控制需要指定
        #rosservice call /controller_manager/switch_controller "stop_controllers: ['scaled_pos_joint_traj_controller']"
        #rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']"
        msg=Float64MultiArray()
        #开始执行轨迹
        for  i in range(self.waypoints.shape[1]):
            waypoint_data = self.waypoints[:,i]
            #顺序又变啦['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            msg.data = [0.0,-waypoint_data[0],-waypoint_data[1],-waypoint_data[2],0.0,0.0]
            #msg.data=[0,0,0,0.1,0,0]
            #msg.layout.data_offset=1
            self.vel_pub.publish(msg)
            self.rate.sleep()
        #停止机械臂
        msg.data=[0,0,0,0,0,0]
        for _ in range(50):
            self.vel_pub.publish(msg)
            time.sleep(0.1)
        
        joint_states = rospy.wait_for_message("joint_states",JointState)
        joints_pos = joint_states.position
        print("Current Joints: {}".format(joints_pos))
        
        
       
        


    def smq_move_position(self):
            #rosservice call /controller_manager/switch_controller "stop_controllers: ['joint_group_vel_controller']"
            #rosservice call /controller_manager/switch_controller "start_controllers: ['scaled_pos_joint_traj_controller']"


            #从joint_state话题上获取当前的关节角度值，因为后续要移动关节时第一个值要为当前的角度值
            joint_states = rospy.wait_for_message("joint_states",JointState)
            joints_pos = joint_states.position
            #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
            goal = FollowJointTrajectoryGoal()
            #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
            goal.trajectory = JointTrajectory()
            #goal.trajectory底下一共还有两个成员，分别是joint_names和points，先给joint_names赋值
            goal.trajectory.joint_names = joint_states.name

            #print("Joint Names: {}".format(joint_states.name))
            #print(" Joints: {}".format(joints_pos))

            #第一点需要是当前位置点
            goal.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(0.0)))
            #第二点设定回到初始位置，
            init_pose = list(joints_pos)
            #theta1    'shoulder_lift_joint'
            init_pose[1] = -self.init_angle[0]
            #theta2    'elbow_joint'
            init_pose[0] = self.init_angle[0]-self.init_angle[1]
            #theta3    'wrist_1_joint'
            init_pose[3] = self.init_angle[1]-self.init_angle[2]-pi/2
            #shoulder_pan_joint
            init_pose[2] = 0.0
            #'wrist_2_joint'
            init_pose[4] = 0.0
            #'wrist_3_joint'
            init_pose[5] = 0.0

            init_time = 10.0
            goal.trajectory.points.append(JointTrajectoryPoint(positions=init_pose, velocities=[0]*6,time_from_start=rospy.Duration(init_time)))
            waypoints_pose = init_pose

            #开始执行轨迹
            for  i in range(self.waypoints.shape[1]):
                waypoint_data = self.waypoints[:,i]
                #各个关节角速度
                joints_velocities =[-waypoint_data[1],-waypoint_data[0],0.0,-waypoint_data[2],0.0,0.0]
                #计算近似关节角
                approx_position = [a+b*self.step    for a,b in zip(waypoints_pose,joints_velocities)  ]
                #更新路点关节角
                waypoints_pose = approx_position
                #执行时间间隔
                duration_time = (i+1)*self.step+init_time
                goal.trajectory.points.append(JointTrajectoryPoint(positions=approx_position, velocities=joints_velocities,time_from_start=rospy.Duration(duration_time)))

            #发布goal，注意这里的client还没有实例化，ros节点也没有初始化，我们在后面的程序中进行如上操作
            self.position_control_client.send_goal(goal)
            self.position_control_client.wait_for_result()
            print("Current Joints: {}".format(joints_pos))


    def move(self):
            #从joint_state话题上获取当前的关节角度值，因为后续要移动关节时第一个值要为当前的角度值
            joint_states = rospy.wait_for_message("joint_states",JointState)
            joints_pos = joint_states.position
            #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
            goal = FollowJointTrajectoryGoal()
            #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
            goal.trajectory = JointTrajectory()
            #goal.trajectory底下一共还有两个成员，分别是joint_names和points，先给joint_names赋值
            goal.trajectory.joint_names = joint_states.name

            print("Joint Names: {}".format(joint_states.name))
            print(" Joints: {}".format(joints_pos))

            #给trajectory中的第二个成员points赋值
            #points中有四个变量，positions,velocities,accelerations,effort，我们给前三个中的全部或者其中一两个赋值就行了
            goal.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(0.0)))
            time = 0
            for  i in range(6):
                joints_pos_i = list(joints_pos)
                joints_pos_i[i] +=0.5
                time_i =time+ 5.0
                time_j =time+ 10.0
                time = time_j

                goal.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos_i, velocities=[0]*6,time_from_start=rospy.Duration(time_i)))
                goal.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(time_j)))
           
            
            goal.trajectory.points.append(JointTrajectoryPoint(positions=(pi/2, \
                -2*pi/3, 0.0, \
                    -1.2874587217914026, -pi/2, 0.0), velocities=[0]*6,time_from_start=rospy.Duration(time+5)))

            #发布goal，注意这里的client还没有实例化，ros节点也没有初始化，我们在后面的程序中进行如上操作
            self.position_control_client.send_goal(goal)
            self.position_control_client.wait_for_result()
            print("Current Joints: {}".format(joints_pos))
    


    def read_xls(self,path):
        data = xlrd.open_workbook(path)
        table = data.sheets()[0]
        nrows = table.nrows  # 行数
        ncols = table.ncols  # 列数
        datamatrix = np.zeros((nrows, ncols))
        for i in range(ncols):
            cols = table.col_values(i)
            datamatrix[:, i] = cols
        return datamatrix


 
if __name__ == "__main__":
    #waypoints_data_path = "/home/wgk/Ubuntu_share/optimal_control_input.xls"
    #waypoints_data_path = "/home/wgk/Ubuntu_share/our_control_input.xls"
    #waypoints_data_path = "/home/wgk/Ubuntu_share/delay_control_input.xls"

    #waypoints_data_path = "/home/wgk/Ubuntu_share/optimal_control_input_trajectory1.xls"
    #waypoints_data_path = "/home/wgk/Ubuntu_share/our_control_input_trajectory1.xls"
    waypoints_data_path = "/home/wgk/Ubuntu_share/delay_control_input_trajectory1.xls"

    init_angle = [pi/2+math.asin(2/math.sqrt(5)),\
                                pi/2+math.asin(1/math.sqrt(5)),\
                                math.asin(1/math.sqrt(10))]
    time_step = 0.1 
    smq_demo = SMQ(waypoints_data_path,init_angle,time_step,sim=False)