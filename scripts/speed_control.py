#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np


vel_msg = Twist()

x = y = z = 0.0
roll = pitch = yaw = 0.0

class Turtlebot3():

    def __init__(self):
        rospy.init_node('turtlebot3_motion',anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom',Odometry, self.odom_callback)
        rospy.spin()
        
    
    def odom_callback(self,msg):
        global x,y,z,roll,pitch,yaw
        (x,y,z) = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print("roll = %.2f  pitch= %.2f yaw = %.2f"%(roll, pitch, yaw))
        print("\nx = %.2f  y= %.2d z = %.2f"%(x, y, z))

Turtlebot3()


    

        
# if __name__ == '__main__':

#     try:

#         #Düğüm Oluştma

#         #Publisher bildirme

#         rospy.Publisher("")
#         self./* pub_name */ = rospy.Publisher('/* topic_name */', /* msg_type */, queue_size=10)
    
#     except rospy.rospy.ROSInterruptException:
#         rospy.loginfo("node terminated.")