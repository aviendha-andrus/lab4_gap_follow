#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.scan_subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,               
            self.lidar_callback,
            10
            )
        self.scan_subscription  

        # TODO: Publish to drive
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, 
            drive_topic, 
            10
            )
        
        # command line parameters 

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # not using highest_value right now
        # why would you use highest value and set higher values to np.inf? 
        # look at the top left corner moving clockwise for an example. 
            # when it sights the longest distance as the corner it moves 
            # towards the longest distance so it doesn't go around the bend. 
            # this causes other problems of it running into a wall thus not using it. 

        highest_value = 5.2
        window = 10

        # 1. set value in 'ranges' to mean (over some window?) (what number)
        tmp = np.ones(window) / window
        proc_ranges = np.convolve(ranges, tmp, mode='same')

        # 2. reject high values 
        # proc_ranges[proc_ranges > highest_value] = np.inf

        # SET NANS AND INFS 

        # set data behind to 0. [270:810]
        proc_ranges[0:270] = 0
        proc_ranges[810:1080] = 0

        return proc_ranges


    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges """
        # this function finds the longest gap 
        # if the space is free, meaning the distance ahead is non-zero, 
        # we either start counting a gap or add to the gap length 
        # if the gap is over, we decide if that gap was the largest

        max_gap = 0
        current_gap = 0
        start_index = -1
        end_index = -1
        current_start_index = None
        
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 0:        # if space is free
                if current_start_index is None: # if gap isn't started
                    current_start_index = i     # start gap
                current_gap += 1                # add to gap length

            else:
                if current_start_index is not None: # gap ended
                    if current_gap > max_gap:       # is it the largest gap?
                        max_gap = current_gap
                        start_index = current_start_index
                        end_index = i - 1  
                    current_start_index = None
                    current_gap = 0
        
        # edge case - end of array 
        if current_start_index is not None:
            if current_gap > max_gap:
                max_gap = current_gap
                start_index = current_start_index
                end_index = len(free_space_ranges) - 1

        # IMPLEMENT GO STRAIGHT INSTEAD OF CRASHING WHEN NO GAP
        # we only need this when we set mgv (max gap variable) to greater than 0
        # it is neccisary because if there is no gap where everythings > mgv
        # the function would provide -1, -1 and the program would crash 
        if (start_index == -1 | end_index == -1):
            start_index = 540
            end_index = 540
            print(f"------ OUCH OUCH OUCH OUCH OUCH -------")
        
        # error checking 
        print(f"gap from index {start_index} to {end_index} with length {max_gap}.")
        print(f"{start_index}: {free_space_ranges[start_index]} {end_index}:{free_space_ranges[end_index]}.")
        return (start_index, end_index)
      
    
    def find_best_point(self, start_i, end_i, ranges):

        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # # check the passed are in range 
        # if start_i < 270 or end_i > 810 or start_i > end_i:
        #     print(f"\nERROR WHAT ERROR WHAT\n")
        #     return None

        # we currently return the longest point
        # within the index's passed (which should be the largest gap)
        # the middle doesn't work very well. 
        # why we might consider the middle - to "look ahead" 
            # for example, in the top lefthand corner moving clockwise 
            # the car sees the longest distance and heads for it thus crashing
        # naive = start_i + np.argmax(ranges[start_i:end_i + 1])
        # middle = (start_i + end_i) // 2  # choose the middle index?
        # print(f"naive best point at {naive}")
        # print(f"middle best point at {middle}")
        # return naive

        # add comments 
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(80), 'same') / 80
        print(f"best point at {averaged_max_gap.argmax() + start_i}")
        return averaged_max_gap.argmax() + start_i


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        # preprocess range data 
        proc_ranges = self.preprocess_lidar(ranges)

        # Find closest point to LiDAR
        # we want the index of the lowest non-zero value
        # we also want the index's of the array to remain the same 
        # so we change the 0's to infinity, allowing us to search for 
        # the lowest value thats non-zero
        searchable_arr = np.where(proc_ranges == 0, np.inf, proc_ranges)
        closest_index = np.argmin(searchable_arr)

        # for testing
        print(f"closest index: {closest_index}")
        print(f"{closest_index}: {proc_ranges[closest_index]}")

        # Eliminate all points inside 'bubble' (set them to zero) 
        # we draw a bubble of length bubble_radius around the closest point
        # by deciding how many scans to cover 
        # we then get the starting and ending index so that we can 
        # 0 out everything in that bubble 
        # if our index's arn't in range we just return left and right values
        # bubble_radius should be a command line param. very important for tuning
        bubble_radius = .4 # used to be .7
        bubble = int(bubble_radius / data.angle_increment) 
        start_idx = np.where(closest_index - bubble > 270, closest_index - bubble, 270)
        end_idx = np.where(closest_index + bubble + 1 < 810, closest_index + bubble + 1, 810)
        proc_ranges[start_idx:end_idx] = 0 

        # project out both ways by r 

        # radious r 
        # closest point 

        # i - closest point 
        # figure out the length in between each scan point 
        # subtract that point from r 
        



        # for testing
        print(f"bubble: {np.arange(start_idx, end_idx)[proc_ranges[start_idx:end_idx] == 0]}")

        # disparity extend 
        # not really working. 
        # if succesive lidar scans have a large disparity, extend the walls 
        # iterate through laser scan. if one scan to the next exceeds the threshold
        # take the smaller distance, and the farther index
        # and overwrite the farther distance with the closer one (or 0?)

        # right NOW the if statement never gets tripped 

        # change algorithm so that I extend in the CORRECT direction 
        # extend in the direction where the difference is larger 

        # make sure i - extend and i + extend will be in range 
        threshold = 0.5   # the disparity threshold 
        extend = 5      # number of scans to overwrite
        for i in range(1, len(proc_ranges)):
            if abs(proc_ranges[i] - proc_ranges[i - 1]) > threshold:
                if (proc_ranges[i] > proc_ranges[i - 1]): # if index [207] > [208]
                    proc_ranges[i:i - extend] = proc_ranges[i-1]
                else:
                    proc_ranges[i - 1 - extend: i] = proc_ranges[i]

                # closer_distance = min(proc_ranges[i], proc_ranges[i - 1])
                # farther_index = i if proc_ranges[i] > proc_ranges[i - 1] else i - 1
                # print(f"farther_index: {farther_index} [{proc_ranges[farther_index]}]), to, index: {farther_index + extend}")
                # proc_ranges[farther_index:farther_index + extend] = closer_distance

        #Find max length gap 
        start_index, end_index = self.find_max_gap(proc_ranges)
        
        #Find the best point in the gap 
        best_point = self.find_best_point(start_index, end_index, proc_ranges)
        
        # print(f"extend: {np.arange(farther_index, farther_index + extend)[proc_ranges[farther_index:farther_index + extend] == closer_distance]}")
        # Calculate the drive angle
        angle = (data.angle_increment * best_point) + data.angle_min

        velocity = 0.0


        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle 
        drive_msg.drive.speed = velocity 
        self.publisher_.publish(drive_msg) 


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()