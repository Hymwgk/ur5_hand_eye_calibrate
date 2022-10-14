#!/usr/bin/env python
#coding=utf-8
from email import header
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import *
from control_msgs.msg import *
from std_msgs.msg import String, Float64MultiArray
import math
import pickle
import message_filters
import numpy as np


class Listener():


    def __init__ (self):
        #self.save_path = './optimal.pkl'
        #self.save_path = './our.pkl'
        #self.save_path = './delay.pkl'


        #self.save_path = './optimal1.pkl'
        #self.save_path = './our1.pkl'
        self.save_path = './delay1.pkl'


        
        self.final_data = np.empty([0,10],dtype=np.float32)
        self.stop_count =0


        rospy.init_node('listener', anonymous=True)
        t1= message_filters.Subscriber("/joint_states", JointState)
        t2 =message_filters.Subscriber("/joint_group_vel_controller/command", Float64MultiArray)
        ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
        ts.registerCallback(self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()



    def callback(self, data1, data2):
        time  =  float(str(data1.header.stamp.secs)+'.'+str(data1.header.stamp.nsecs))
        # ['elbow_joint','shoulder_lift_joint','shoulder_pan_joint',  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_name           = data1.name
        real_joint_angle = data1.position
        real_joint_vel      = data1.velocity

        target_joint_vel = list(data2.data)
        #重新排序

        real_theta_vel =np.array([ -real_joint_vel[1],  -real_joint_vel[0], -real_joint_vel[3] ],dtype=np.float32)
        ## ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        target_theta_vel =np.array([-data2.data[1],-data2.data[2],-data2.data[3]],dtype=np.float32)


        real_theta_angle =   np.array([0,0,0],dtype=np.float32)
        real_theta_angle[0] = -real_joint_angle[1]
        real_theta_angle[1] = real_theta_angle[0]-real_joint_angle[0]
        real_theta_angle[2] = real_theta_angle[1]-real_joint_angle[3]- math.pi/2

        #[time,target_theta1_vel,target_theta2_vel,target_theta3_vel,real_theta1_vel,real_theta2_vel,real_theta3_vel,real_theta1_angle,real_theta2_angle,real_theta3_angle  ]
        waypoint = np.concatenate((np.array([time],dtype=np.float32),target_theta_vel,real_theta_vel,real_theta_angle),axis=0).reshape(1,10)
        self.final_data = np.concatenate((self.final_data,waypoint),axis=0)

        if target_theta_vel.sum() == 0:
            self.stop_count +=1

        if self.stop_count>20:
            f=open(self.save_path,'w') 
            pickle.dump(self.final_data,f)


        print(target_theta_vel)
        print(real_theta_vel)
        print(real_theta_angle)
        print(waypoint)
        print('==============')
        


if __name__ == '__main__':
    listener = Listener()
