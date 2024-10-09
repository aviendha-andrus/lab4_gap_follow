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

        highest_value = 3
        window = 5

        # sliced_ranges = ranges[270:811]
        
        sliced = ranges[270:810]
        # print(len(sliced))

        # 1. set value in 'ranges' to mean (over some window?) (what number)
        tmp = np.ones(window) / window
        proc_ranges = np.convolve(sliced, tmp, mode='same')

        # 2. reject high values 
        proc_ranges[proc_ranges > highest_value] = 0

        # SET NANS AND INFS 

        return proc_ranges

        """alg for max_gap"""
        # largest number of consequitive non-zero 
        # iterate over the array 
        # positive spaces are free 

        # for i in free space range 
            # if free space (distance positive)
                # if a gap isn't started
                    # start gap here 
                # else add to gap length

            # else gap ended
                # check the length of the gap 
                # if its larger than max gap length set 
                    # max gap length 
                    # max start 
                    # max end
                # reset current start 
                # reset current gap length 
        # edge case 
        # return None

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges """
        max_gap = 0
        current_gap = 0
        max_start_index = -1
        max_end_index = -1
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
                        max_start_index = current_start_index
                        max_end_index = i - 1  
                    current_start_index = None
                    current_gap = 0
        
        # edge case - end of array 
        if current_start_index is not None:
            if current_gap > max_gap:
                max_gap = current_gap
                max_start_index = current_start_index
                max_end_index = len(free_space_ranges) - 1
        
        # for debuging 
        # print index 
        # and the stuff in them 
        print(f"gap from index {max_start_index} to {max_end_index} with length {max_gap}.")
        return (max_start_index, max_end_index)
    
      
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # check the passed are in range best_point = self.find_best_point(start_index, end_index, proc_ranges)

        if start_i < 0 or end_i >= len(ranges) or start_i > end_i:
            return None

        naive = start_i + np.argmax(ranges[start_i:end_i + 1])
        # middle = (start_i + end_i) // 2  # choose the middle index?

        print(f"best point at {naive}")
        return naive

    def disparity_extender(self, proc_ranges):
        """ Apply the disparity extender algorithm to filter the LIDAR ranges. """

    

    
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        # preprocess range data 
        proc_ranges = self.preprocess_lidar(ranges)

        # Filter ranges 

        #Find closest point to LiDAR
        valid_ranges = proc_ranges[proc_ranges > 0] 
        closest_index = np.argmin(valid_ranges)

        # # Eliminate all points inside 'bubble' (set them to zero) 
        # # bubble surrounds closest point
        # bubble_radius = .35
        # # DRAW BUBBLE 
        # # zero out everything in that bubble 

        # bubble = int(bubble_radius / data.angle_increment)  # Assuming angle resolution of 0.1 radians
        # start_idx = max(0, closest_index - bubble)
        # end_idx = min(len(proc_ranges), closest_index + bubble + 1)
        # proc_ranges[start_idx:end_idx] = 0


        # disparity extend 
        # if succesive lidar scans have a large disparity, extend the walls 
        threshold = 4   # the disparity threshold 
        extend =  .14   # amount to extend the walls by 
        # proc_ranges = self.disparity_extender(proc_ranges)
        for i in range(1, len(proc_ranges)):
            if abs(proc_ranges[i] - proc_ranges[i - 1]) > threshold:
                closer_distance = np.min(proc_ranges[i], proc_ranges[i - 1])
                farther_index = i if proc_ranges[i] > proc_ranges[i - 1] else i - 1
                # overrite
                for j in range(farther_index, min(farther_index + extend, len(proc_ranges))):
                    if proc_ranges[j] > closer_distance:
                        proc_ranges[j] = 0

        #Find max length gap 
        start_index, end_index = self.find_max_gap(proc_ranges)
        
        #Find the best point in the gap 
        best_point = self.find_best_point(start_index, end_index, proc_ranges)

        # Calculate the drive angle
        angle = (data.angle_increment * (best_point + 270)) + data.angle_min

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