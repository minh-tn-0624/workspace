#!/usr/bin/env python
from __future__ import print_function

# from cmath import atan, cos, pi, sin

import sys
#import math
from math import *
from xml.etree.ElementTree import tostring
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive #publish steering_angle

from std_msgs.msg import Float64


#PID control params
kp = 2
kd = 0.0
ki = 0.00002


# disired params
VELOCITY = 1.5
DESIRED_DISTANCE_RIGHT = 1.0

#system params
frequency = 50.0   #10Hz

class WallFollow():
    def __init__(self):   #define Lab1Class constructure
        rospy.init_node("wallfollow_node")  #name of the NODE

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback) 

        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size =10 )
        # use button "n" to allow the Behavior Controller to switch the Mux to /nav 

        self.a = 0.0 #distence to wall with theta angle (theta = 60 degree)
        self.b = 0.0 #distance to wall with zero degree angle

        # self.rate = rospy.Rate(frequency) 
        self.rate = 1.0/frequency

        # self.then = rospy.Time.now() #save when last time callback function was called

        #PID info
        self.current_error = 0.0
        self.prev_error = 0.0
        self.sum_prev_error = 0.0

        self.steering_angle = 0.0

        
    def rightRange(self, data):    
        total_beams = len(data.ranges)
        max_angle = data.angle_max
        min_angle = data.angle_min

        total_cover_angle = max_angle-min_angle
        step_scan = total_cover_angle/total_beams

        theta = pi/18 #10 degree

        self.a = data.ranges[int(total_beams/2 - 1.57/step_scan + theta/step_scan)]
        self.b = data.ranges[int(total_beams/2 - 1.57/step_scan)] #right of car
        # print ("b_right = " + str(self.b) )

        anpha = atan2((self.a*cos(theta)-self.b),(self.a*sin(theta)))
        print ("anpha_right = " +str(anpha))
        D_right = self.b*cos(anpha)
        
        return D_right

    def leftRange(self, data):    
        total_beams = len(data.ranges)
        max_angle = data.angle_max
        min_angle = data.angle_min

        total_cover_angle = max_angle-min_angle
        step_scan = total_cover_angle/total_beams

        theta = pi/18 #10 degree

        self.a = data.ranges[int(total_beams/2 + 1.57/step_scan-theta/step_scan)]
        self.b = data.ranges[int(total_beams/2 + 1.57/step_scan)] #left of car
        # print ("b_left = " + str(self.b) )

        anpha = atan2((self.a*cos(theta)-self.b),(self.a*sin(theta)))
        print ("anpha_left = " +str(anpha))
        D_left = self.b*cos(anpha)
        
        return D_left  

    def pid_control(self, curr_error, velocity):
        global kp
        global ki
        global kd

        #dt = time differential
        dt = self.rate

        p_gain = kp*curr_error

        d_gain = kd*((curr_error-self.prev_error)/dt)

        i_gain = self.sum_prev_error + ki*curr_error*dt

        #use PID params to compute angle
        self.steering_angle = p_gain + d_gain + i_gain

        #save current info to previous info
        self.prev_error = curr_error
        self.sum_prev_error = i_gain

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = self.steering_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        print ("publish is called")    


    def lidar_callback(self, data):  


        # now = rospy.Time.now()
        # dt = now - self.then
        # dt = dt.to_sec()
        # if dt < 0.001:  #dt < 1/frequency
        #     return
       
        
            
        D_right = WallFollow.rightRange(self, data)
        print ("distance to right wall = " + str(D_right))

        D_left = WallFollow.leftRange(self, data)
        print ("distance to left wall = " + str(D_left))
        # print ("middle = " + str((D_right+D_left)/2))
        DESIRED_DISTANCE_RIGHT = (D_right+D_left)/1.5
        self.current_error = DESIRED_DISTANCE_RIGHT - D_right
    
        # self.pid_control(current_error, VELOCITY) 
            
        # self.then = now

    def run(self):
        rospy.Timer(rospy.Duration(self.rate), self.control_with_rate)
        rospy.spin()
        # while not rospy.is_shutdown():
        #     self.rate.sleep()

    def control_with_rate(self, event):
        VELOCITY = 1.5 - min(1.0,self.current_error*5,self.steering_angle*1.2)
        self.pid_control(self.current_error, VELOCITY) 


# lab1 = WallFollow()
# rospy.spin()

if __name__ == '__main__':
    wfh = WallFollow()
    wfh.run()