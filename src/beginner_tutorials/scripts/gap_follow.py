#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

bubble_radius = 0.5 #meter




class reactive_follow_gap:
    def __init__(self):
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

    
    
    def find_closest_point(self,ranges):
        closest_distance = ranges[0]
        closest_point_idx = 0
        for idx in range(len(ranges)):
            if closest_distance > ranges[idx]:
                closest_distance = ranges[idx]
                closest_point_idx = idx
        return closest_point_idx, closest_distance
    
    def set_safety_bubble(self, ranges, closest_point_idx, closest_distance):

        anpha = atan(bubble_radius/closest_distance) #field of lidar view to bubble

        N = int(2*anpha/self.step_scan) #number of element of ranges belong to bubble
        # N = int(2*anpha/0.2) 
        ranges_with_bubble = list(ranges)

        for i in range(N):
            ranges_with_bubble[min(closest_point_idx+i-int(N/2),len(ranges)-1)] = 0.0

        return ranges_with_bubble

    def find_max_gap(self, free_space_ranges):
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
        #Closest distance = 2:
        furthest_distance = ranges[start_idx]
        furthest_distance_idx = start_idx
        for idx in range(start_idx,end_idx + 1):
            if furthest_distance < ranges[idx]:
                furthest_distance = ranges[idx]
                furthest_distance_idx = idx
        best_point_idx = furthest_distance_idx
        best_point = furthest_distance

        
        return best_point, best_point_idx
    
    def get_steering_angle_from_the_best_point(self, ranges, best_point_idx):
        steering_angle = float((best_point_idx - len(ranges)/2.0))*self.step_scan
        # print ("steering angle = " + str(steering_angle))
        return steering_angle
    
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        
        #get lidar_params
        self.total_beams = len(data.ranges)
        self.max_angle = data.angle_max
        self.min_angle = data.angle_min
        self.total_cover_angle = self.max_angle - self.min_angle
        self.step_scan = self.total_cover_angle/self.total_beams
       
       
       
       
       
       
        #Find closest point to LiDAR
        closest_point_idx, closest_distance = self.find_closest_point(ranges)
        
        
        #Eliminate all points inside 'bubble' (set them to zero) 
        ranges_with_bubble = self.set_safety_bubble(range, closest_point_idx, closest_distance)
        
        
        #Find max length gap 
        start_idx, end_idx = self.find_max_gap(ranges_with_bubble)
        
        
        
        #Find the best point in the gap 
        best_point_idx = self.find_best_point(start_idx, end_idx, ranges)
        

        #Publish Drive message
        angle = self.get_steering_angle_from_the_best_point(ranges,best_point_idx)
        velocity = 1
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

def main(args):
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)