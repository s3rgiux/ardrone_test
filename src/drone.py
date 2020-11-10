#!/usr/bin/env python
# coding: utf-8

import numpy as np
import rospy
import tf
#from tf.transformations import quaternion_matrix
#from sensor_msgs.msg import Image, CameraInfo
#from fiducial_msgs.msg import FiducialArray, FiducialTransformArray
#from cv_bridge import CvBridge
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose, TransformStamped,Quaternion
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Empty


class Drone: 
    

    def __init__(self):
        rospy.init_node('DroneNode', log_level=rospy.INFO)
        rospy.on_shutdown(self.on_shutdown)
        self.odom_sub = rospy.Subscriber('ardrone/odometry', Odometry, self.callback)
        self.joy_sub = rospy.Subscriber('j0/joy', Joy, self.joy_callback)
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tkof_pub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('ardrone/land', Empty, queue_size=1)
        self.x=0.0
        self.y=0.0
        self.z=0.0
        self.ang= Quaternion()
        self.cmd_v=Twist()
        self.cmd_tkof=Empty()
        self.cmd_land=Empty()
        self.autonomous=False
        self.state = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        #self.pub = rospy.Publisher('aruco/image', Image, queue_size=1)


    def joy_callback(self, joy):
        axes = joy.axes
        forward = axes[1]
        yaw = axes[0]
        up = axes[4]
        side= axes[3]
        tkof=joy.buttons[2]
        lnd=joy.buttons[0]
        auto= joy.buttons[1]
        if tkof==1:
            self.tkof_pub.publish(self.cmd_tkof)
            self.state=0
        elif lnd==1:
            self.land_pub.publish(self.cmd_land)
            self.state=0
        if auto==1 and self.autonomous==False:
            self.autonomous=True
        #elif auto==1 and self.autonomous==True:
        #    self.autonomous=False 
        if self.autonomous==False:
            self.state=0
            self.cmd_v.linear.x=forward
            self.cmd_v.linear.y=side
            self.cmd_v.linear.z=up
            self.cmd_v.angular.z=yaw
            self.move_pub.publish(self.cmd_v)

    def callback(self, odom):
        self.x=odom.pose.pose.position.x
        self.y=odom.pose.pose.position.y
        self.z=odom.pose.pose.position.z
        euler = tf.transformations.euler_from_quaternion(odom.pose.pose.orientation)
        self.roll=euler[0]
        self.pitch=euler[1]
        self.yaw=euler[2]
        print(self.yaw)
        if self.autonomous==True:
            if self.state==0:
                zd=1.0
                yawd=0.0
                xd=0.0
                yd=0.0
                ez=zd-self.z
                eyaw=yawd-self.yaw
                ctrlz=ez*1.2
                ctrlyaw=eyaw*1.2
                ex=xd-self.x
                ey=yd-self.y
                ctrlx=ex*1.2
                ctrly=ey*1.2
                if eyaw<0.2 and ez < 0.15 and ex<0.2 and ey <0.2:
                    self.state=1
            if self.state==1:
                zd=1.0
                yawd=0.0
                xd=1.5
                yd=0.0
                ez=zd-self.z
                eyaw=yawd-self.yaw
                ctrlz=ez*1.2
                ctrlyaw=eyaw*1.2
                ex=xd-self.x
                ey=yd-self.y
                ctrlx=ex*1.2
                ctrly=ey*1.2
                if eyaw<0.2 and ez < 0.15 and ex<0.2 and ey <0.2:
                    self.state=2
            if self.state==2:
                self.land_pub.publish(self.cmd_land)
                self.state=0  

            self.cmd_v.linear.x=ctrlx
            self.cmd_v.linear.y=ctrly
            self.cmd_v.linear.z=ctrlz
            self.cmd_v.angular.z=ctrlyaw
            self.move_pub.publish(self.cmd_v)
        #self.yaw=odom.pose.pose.orientation

    def on_shutdown(self):
        return
        

if __name__ == '__main__':
    node = Drone()
    # 制御周期
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    # [ctrl]+[c]でプログラムの終了するまでループ
    while not rospy.is_shutdown():
        R.sleep()
