#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        rospy.init_node("FollowGap_node",anonymous=True)

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback) 
        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size =10 )

        #Lidar params
        self.total_beams = 0
        self.max_angle = 0
        self.min_angle = 0
        self.total_cover_angle = 0.0
        self.step_scan = 0.0
    # def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # proc_ranges = []
        # for x in range(len(ranges)):
        #     if 
        # proc_ranges = ranges
        # return proc_ranges

    
    
    
    
    
    
    
    
    def find_closet_point(self,ranges):
        closest_distance = ranges[0]
        closest_point_idx = 0
        for idx in range(len(ranges)):
            if closest_distance > ranges[idx]:
                closest_distance = ranges[idx]
                closest_point_idx = idx
        return closest_point_idx, closest_distance
    
    # def set_safety_bubble(self, ranges, closest_point_idx, closest_distance):


    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap = []
        curr_max_gap = [] 
        start_idx = 0
        end_idx = 0
        for idx in range(len(ranges)):
            if ranges[idx] > 0.0:
                curr_max_gap.append(ranges[idx])
            else:
                if len(curr_max_gap) > len(max_gap):
                    max_gap = curr_max_gap
                    curr_max_gap = []
                    end_idx = idx - 1
                    start_idx = end_idx - len(max_gap) + 1

        return start_idx, end_idx
    
    def find_best_point(self, start_idx, end_idx, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        #Closet distance = 2:
        furthest_distance = ranges[start_idx]
        furthest_distance_idx = start_idx
        for idx in range(start_idx,end_idx + 1):
            if furthest_distance < ranges[idx]:
                furthest_distance = ranges[idx]
                furthest_distance_idx = idx
        best_point_idx = furthest_distance_idx
        best_point = furthest_distance

        return best_point, best_point_idx
    
    # def get_steering_angle_from_the_best_point(self, ranges, best_point_idx):
        # 
        # 
        # 

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)