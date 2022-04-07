#!/usr/bin/env python
from __future__ import print_function
import sys
from math import *
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

bubble_radius = 0.2 #meters

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

class reactive_follow_gap:
    def __init__(self):
        rospy.init_node("FollowGap_node", anonymous=True)  #name of the NODE

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback) 

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 10 )
        
        #lidar params
        self.total_beams = 0
        self.max_angle = 0.0
        self.min_angle = 0.0
        self.total_cover_angle = 0.0
        self.step_scan = 0.0

        #control params
        self.angle = 0.0
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges=[]

        # for i in range(len(ranges)):
        #     if (i < (int(self.total_beams/2 - 1.57/self.step_scan))) or (i > int((self.total_beams/2 + 1.57/self.step_scan))):
        #         proc_ranges.append(1.0)
        #     else:
        #         proc_ranges.append(ranges[i])
        for i in range(len(ranges)):
            if (i > (int(self.total_beams/2 - 1.57/self.step_scan)) + 45 ) and (i < int((self.total_beams/2 + 1.57/self.step_scan)) - 45):
                proc_ranges.append(ranges[i])
        return proc_ranges

    def find_closest_point(self, ranges):
        closest_distance = ranges[0]
        closest_point_idx = 0
        for idx in range(len(ranges)-1):
            if closest_distance > ranges[idx]:
                closest_distance = ranges[idx]
                closest_point_idx = idx
        return closest_point_idx, closest_distance
    
    def set_safety_bubble(self, ranges, closest_point_idx, closest_distance):

        anpha = atan(bubble_radius/closest_distance) #field of lidar view to bubble

        N = int(2*anpha/self.step_scan) #number of element of ranges belong to the bubble
        # N = int(2*anpha/0.2) 
        ranges_with_bubble = list(ranges)

        for i in range(N):
            ranges_with_bubble[min(closest_point_idx+i-int(N/2),len(ranges)-1)] = 0.0

        return ranges_with_bubble

    def find_max_gap(self, ranges):
        max_subset = []
        current_max_subset = []
        start_index = 0
        end_index = 0
        ranges[len(ranges)-1] = 0
        for idx in range(len(ranges)):
            if ranges[idx] > 1.8:
                current_max_subset.append(ranges[idx])
            else:
                if len(current_max_subset) > len(max_subset):
                    max_subset = current_max_subset
                    end_index = idx - 1 #previous element is the end of list
                current_max_subset = [] #reset list to find another gap
        start_index = end_index - len(max_subset) + 1
        # print(max_subset)
        return start_index, end_index
    
    def find_best_point(self, ranges , start_index, end_index):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # # if closest_distance < 4:
        # furthest_distance = ranges[start_index]
        # furthest_point_idx = start_index
        # for idx in range(start_index, end_index + 1):
        #     if furthest_distance < ranges[idx]:
        #         furthest_distance = ranges[idx]
        #         furthest_point_idx = idx
        # best_point_idx = furthest_point_idx
        # print ("the furthest point in gap = " +str(furthest_distance))
        # else
        """if the car is away 3-4m from the closest point, let the car goes to 
        the start index (obstacle on the left) or end index (obstacle on the right)"""
        best_point_idx = int((start_index + end_index)/2)


        return best_point_idx
    
    def get_steering_angle_from_the_best_point(self, ranges, best_point_idx):
        steering_angle = float((best_point_idx - len(ranges)/2.0))*self.step_scan
        # print ("steering angle = " + str(steering_angle))
        return clamp(steering_angle,-1.57,1.57)
    
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        # self.angle = 0.0
        #get lidar params
        self.total_beams = len(data.ranges)
        self.max_angle = data.angle_max
        self.min_angle = data.angle_min
        self.total_cover_angle = self.max_angle-self.min_angle
        self.step_scan = self.total_cover_angle/self.total_beams

  
        # print("step = " +str(self.step_scan))
        

        ranges = data.ranges
        # test_list = [12.4 , 2.33, 4.55, 34.3, 45, 64, 65, 70, 43, 32, 21, 1.25,
        # 1.24, 1.12, 1.23, 1.25, 1.56, 5, 15, 24, 45, 50 ]

        proc_ranges = self.preprocess_lidar(ranges)
        # print (ranges)
        # print("number elements = " +str(len(proc_ranges)))

        #Find closest point to LiDAR
        closest_point_idex, closest_distance = self.find_closest_point(proc_ranges)
        # print ("closest point idex = " + str(closest_point_idex))
        # print ("closest distance" + str(closest_distance))

        #Eliminate all points inside 'bubble' (set them to zero) 
        ranges_with_bubble = self.set_safety_bubble(proc_ranges, closest_point_idex, closest_distance)
        # print (ranges_with_bubble)

        #Find max length gap 
        start_index, end_index = self.find_max_gap(ranges_with_bubble)
        # print ("start_index = " + str(start_index))
        # print ("end_index = " + str(end_index))

        #Find the best point in the gap 
        best_point_idx = self.find_best_point(proc_ranges, start_index, end_index)
        # print ("best point is " + str(best_point_idx))
        
        #Publish Drive message
        self.angle = self.get_steering_angle_from_the_best_point(proc_ranges,best_point_idx)       

        # print ("steering angle = " + str(self.angle))

        # velocity = 1
        # drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = rospy.Time.now()
        # drive_msg.header.frame_id = "base_link"
        # drive_msg.drive.steering_angle = self.angle
        # drive_msg.drive.speed = velocity
        # self.drive_pub.publish(drive_msg)


    def control_with_rate(self, event):

        # print ("steering angle = " + str(self.angle))
        velocity = 1.0 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = self.angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


    def run(self):
        rospy.Timer(rospy.Duration(1.0/50.0), self.control_with_rate)
        rospy.spin()


if __name__ == '__main__':
    ob = reactive_follow_gap()
    ob.run()
# def main(args):
#     rfgs = reactive_follow_gap()
#     rospy.sleep(0.1)
#     rospy.spin()

# if __name__ == '__main__':
#     main(sys.argv)

