#!/usr/bin/env python
# coding: utf-8

import numpy as np
import rospy
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
        self.x=0
        self.y=0
        self.z=0
        self.ang= Quaternion()
        self.cmd_v=Twist()
        self.cmd_tkof=Empty()
        self.cmd_land=Empty()
        #self.pub = rospy.Publisher('aruco/image', Image, queue_size=1)


    def joy_callback(self, joy):
        axes = joy.axes
        forward = axes[1]
        yaw = axes[0]
        up = axes[4]
        side= axes[3]
        tkof=joy.buttons[2]
        lnd=joy.buttons[0]
        if tkof==1:
            self.tkof_pub.publish(self.cmd_tkof)
        elif lnd==1:
            self.land_pub.publish(self.cmd_land)
        self.cmd_v.linear.x=forward
        self.cmd_v.linear.y=side
        self.cmd_v.linear.z=up
        self.cmd_v.angular.z=yaw
        self.move_pub.publish(self.cmd_v)

    def callback(self, odom):
        self.x=odom.pose.pose.position.x
        self.y=odom.pose.pose.position.y
        self.z=odom.pose.pose.position.z
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
