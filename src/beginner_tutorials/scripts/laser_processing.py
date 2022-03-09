#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from beginner_tutorials.msg import Scan_range

class Lab1Class():
    def __init__(self):   #define Lab1Class constructure
        rospy.init_node("lab1_python")  #name of the NODE

        #create a Subscriber to /scan topic, use messages of LaserScan and call laser_callback
        self.scan_sub_ = rospy.Subscriber('/scan', LaserScan, self.laser_callback) 

        #create a Publisher to a new topic find minimum range call /closet_point
        self.closet_pub_ = rospy.Publisher('/closet_point', Float64, queue_size=10) 

        #create a Publisher to a new topic find maximum range call /farthest_point
        self.farthest_pub_ = rospy.Publisher('/farthest_point', Float64, queue_size=10) 




    def laser_callback(self, msg):
        
        # msgs = LaserScan()
        #take data from msgs
        closet_data = min(msg.ranges)
        closet = Float64()
        closet.data = closet_data

        farthest_data = max(msg.ranges)
        farthest = Float64()
        farthest.data = farthest_data

        #publish these data to new topic
        self.closet_pub_.publish(closet)
        self.farthest_pub_.publish(farthest)
       

lab1 = Lab1Class()
rospy.spin()