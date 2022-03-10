# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from PIL import Image
import scipy.stats
import matplotlib.pyplot as plt
from time import sleep
import numpy as np
import math
import cmath
import time

# constants

TARGETspeedchange = 0.05
TARGETrotatechange = 0.1
TARGET_hotthreshhold = 30.0
TARGETshoot_distance = 0.30
TARGET_front_angle = 3
TARGET_front_angles = range(-TARGET_front_angle,TARGET_front_angle,1)
TARGET_moveres = 0.2 #time for sleep when moving

speedchange = 0.05
rotatechange = 0.1
occ_bins = [-1, 0, 50, 101]
stop_distance = 0.25
initialdelay = 5
front_angle = 20
front_angles = range(-front_angle,front_angle+1,1)


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class Targeter(Node):
    def __init__(self):
        super().__init__('targeter')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        #create subscription to thermal cam array data
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'thermal',
            self.thermal_callback,
            10)

        self.lasersub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data)

        self.subscription  # prevent unused variable warning

        #some data
        self.thermal_array = [0 for i in range(64)]
        self.target_presence = False
        self.front_distance = 100
        self.centered = False


#function that checks through array if there is anything hot
    def detect_target(self):
        max_value = -1.0
        for row in range(len(self.thermal_array)):
            for col in range(len(self.thermal_array[row])):
                current_value = self.thermal_array[row][col]
                if current_value > max_value:
                    max_row = row
                    max_col = col
                    max_value = current_value
        print("max temp = %s @ %s" %(max_value,max_col))
        if max_value >= TARGET_hotthreshhold:     
            self.target_presence = True
        else:
            self.target_presence = False


#function to get thermal arrray data
#check for hot_target
#check centered if not move 1 step to center
#if centered move 1 step forward
    def thermal_callback(self, thermal_array):
        pix_res = (8,8)
        self.get_logger().info('I heard: "%s"' % thermal_array.data)
        self.thermal_array = np.reshape(thermal_array.data,pix_res)
        print(self.thermal_array)
        self.detect_target()
        if self.target_presence == True:
            self.stopbot()
            print("Target Detected, Stop Bot")
            self.center_target()
            if (self.front_distance < TARGETshoot_distance):
                if self.centered == False:
                    self.center_target()
                else:
                    self.stopbot()
                    print("moving onto shooting phase")
                    self.destroy_node(self)
            if self.centered == True and self.front_distance > TARGETshoot_distance:
                    #stop this script and move onto shooting script
                self.robotforward()
        else:
            print("Target Not Detected")
            #return to autonav?
            #thinking of doing a publisher for thermal detection
            pass


    #function to stop bot
    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist) 

    #funtion that moves bot forward by 1 step according TARGETspeedchange variable and sleep time
    def robotforward(self):
        # start moving
        self.get_logger().info('1 step forward')
        twist = Twist()
        twist.linear.x = TARGETspeedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.publisher_.publish(twist)
        twist.linear.x = 0.0
        time.sleep(TARGET_moveres)
        self.publisher_.publish(twist)

    def laser_callback(self, msg):
        # create numpy array
        laser_range = list(msg.ranges)
        # find index with minimum value
        laser_range = laser_range[-TARGET_front_angle:] + laser_range[:TARGET_front_angle + 1]
        print(laser_range)
        laser_range_new = []
        for i in laser_range:
            if i == np.nan or i == np.inf or i == 0:
                continue
            laser_range_new.append(i)
        self.front_distance = np.average(laser_range_new)
        print("This is the front distance")
        print(self.front_distance)

#function checks for hottest region and sends command to rotate by 1 step
    def center_target(self):
        view = self.thermal_array
        max_value = -1.0
        for row in range(len(self.thermal_array)):
            for col in range(len(self.thermal_array[row])):
                current_value = view[row][col]
                if current_value > max_value:
                    max_row = row
                    max_col = col
                    max_value = current_value
        print("max temp = %s @ %s" %(max_value,max_col))
        if max_col < 3:
                # turn 1 step anti-clockwise
                print("Not centered, 1 step ACW")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = TARGETrotatechange
                self.publisher_.publish(twist)
                twist.angular.z = 0.0
                time.sleep(TARGET_moveres)
                self.publisher_.publish(twist)
                self.centered = False
        elif max_col > 4:
                # turn 1 step clockwise
                print("Not centered, 1 step CW")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = -TARGETrotatechange
                self.publisher_.publish(twist)
                twist.angular.z = 0.0
                time.sleep(TARGET_moveres)
                self.publisher_.publish(twist)
                self.centered = False
        else:
            print("Centered and stopped")
            self.stopbot
            self.centered = True


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.map2base_sub = self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            1)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'thermal',
            self.thermal_callback,
            10)

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            1)
        # self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.nfcsubscription = self.create_subscription(Bool,
        'NFC_presence',
        self.NFC_callback,
        10)
        self.buttonsubscription = self.create_subscription(Bool,
        'button_presence',
        self.button_callback,
        10)
        self.resolution = 0.05
        self.nfc_presence = False
        self.button_presence = False
        self.loaded = False
        self.recordedinitial = False
        self.moved_off = False
        self.one_round = False
        self.Xpos = 0
        self.Ypos = 0
        self.XposNoAdjust = 0
        self.YposNoAdjust = 0
        self.Xstart = 0
        self.Ystart = 0
        self.mapbase = 0
        self.thermal_array = []
        self.target_presence = False
        self.mazelayout = []

    def NFC_callback(self,msg):
        if msg.data == True:
            self.nfc_presence = True
        else:
            self.nfc_presence = False

    def button_callback(self,msg):
        if msg.data == True:
            self.button_presence = True
        else:
            self.button_presence = False

    def map2base_callback(self, msg):
        # self.get_logger().info('In map2basecallback')
        
        self.mapbase = msg.position
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 50. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        total_bins = iwidth * iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        self.mazelayout = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        self.Xadjust = msg.info.origin.position.x
        self.Yadjust = msg.info.origin.position.y
        self.Xpos = self.mapbase.x
        self.Ypos = self.mapbase.y
        # self.mazelayout[self.Ypos][self.Xpos] = 6
        
        # img2 = Image.fromarray(self.mazelayout)
        # plt.imshow(img2, cmap='gray', origin='lower')
        # plt.draw_all()
        # # # pause to make sure the plot gets created
        # plt.pause(0.00000000001)


#function that checks through array if there is anything hot        
    def thermal_callback(self, thermal_array):
        pix_res = (8,8)
        self.thermal_array = np.reshape(thermal_array.data,pix_res)
        max_value = -1.0
        for row in range(len(self.thermal_array)):
            for col in range(len(self.thermal_array[row])):
                current_value = self.thermal_array[row][col]
                if current_value > max_value:
                    max_row = row
                    max_col = col
                    max_value = current_value
        print("max temp = %s @ %s" %(max_value,max_col))
        if max_value >= TARGET_hotthreshhold:     
            self.target_presence = True
        else:
            self.target_presence = False


    def scan_callback(self, msg):
        self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def pick_direction(self):
        # self.get_logger().info('In pick direction:')
        
        self.front_dist = np.nan_to_num(
            self.laser_range[0], copy=False, nan=100)
        self.leftfront_dist = np.nan_to_num(
            self.laser_range[45], copy=False, nan=100)
        self.rightfront_dist = np.nan_to_num(
            self.laser_range[315], copy=False, nan=100)

        # self.get_logger().info('Front Distance: %s' % str(self.front_dist))
        # self.get_logger().info('Front Left Distance: %s' % str(self.leftfront_dist))
        # self.get_logger().info('Front Right Distance: %s' % str(self.rightfront_dist))

        # Logic for following the wall
        # >d means no wall detected by that laser beam
        # <d means a wall was detected by that laser beam
        d = 0.28  # wall distance from the robot. It will follow the right wall and maintain this distance
        # Set turning speeds (to the left) in rad/s

        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 0.60  # Fast turn ideal = 1.0
        self.turning_speed_wf_slow = 0.2  # Slow turn = 0.4
        # Set movement speed
        self.forward_speed = speedchange
        # Set up twist message as msg
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow  # turn left to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif (self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d):
            if (self.leftfront_dist < 0.25):
                # Getting too close to the wall
                self.wall_following_state = "turn right"
                msg.linear.x = self.forward_speed
                msg.angular.z = -self.turning_speed_wf_fast
            else:
                # Go straight ahead
                self.wall_following_state = "follow wall"
                msg.linear.x = self.forward_speed

        elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow  # turn left to find wall

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow  # turn left to find wall

        else:
            pass

        # Send velocity command to the robot
        self.publisher_.publish(msg)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def loading(self):
        self.stopbot()
        while self.button_presence == False:
            rclpy.spin_once(self)
        self.loaded = True
        
    def recordposition(self):
        print("######################################")
        print("######################################")
        print("######################################")
        print("######################################")
        print("######################################")
        print("######################################")
        print("######################################")
        print("######################################")
        print("######################################")
        print("######################################")
        print("RECORDED POSITION")
        self.Xstart = self.Xpos
        self.Ystart = self.Ypos


    def move_forward(self):
        # start moving forward
        self.get_logger().info('Moving forward')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(0.2)
        self.publisher_.publish(twist)

    def get_side_wall_distance(self, direction):
      if self.laser_range[direction-5:direction+5] == []:
        return 5
      return np.average(self.laser_range[direction-5:direction+5])
      
    def cut_through(self):
        startingX = ((len(self.mazelayout[0]) * self.resolution) / 2)
        startingY = 0
        print(self.Xadjust) 
        while abs(self.Xpos - startingX) > 0.25 or  abs(self.Ypos - startingY) > 0.25:
            self.pick_direction()
            print("in cut through")
            rclpy.spin_once(self)
            print("target")
            print(startingX)
            print(startingY)
            print("current")
            print(self.Xpos)
            print(self.Ypos)
        self.rotatebot(-90)
        self.move_forward()
        initialY = self.Ypos
        # check prev u-turn direction
        while True:
            self.move_forward()
            while True:
                rclpy.spin_once(self)
                lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                            # self.get_logger().info('Distances: %s' % str(lri))
                            # check if the initial position has been set

                            # if the list is not empty
                if(len(lri[0])>0):
                    # stop moving
                    self.stopbot()
                    self.get_logger().info('Obstacle encountered infront')
                    break
            print("Y initial and difference")
            print(initialY)
            print(abs(initialY - self.Ypos))
            print(len(self.mazelayout) * self.resolution - 2.0)
            if (abs(initialY - self.Ypos) >= len(self.mazelayout) * self.resolution - 2.0):
                break
            bypass_direction = -90
            bypass_opp_dir = 90

            # variable used to hold the current x and y position of the turtlebot
            initialX = self.Xpos
            self.get_logger().info('Initial x position is: %.2f' % initialX)
            # turn the turtlebot before starting adopted Pledge algorithm
            self.get_logger().info('Turtlebot turned %s to start Pledge Algo' % bypass_direction)
            self.rotatebot(bypass_direction)
            # start moving forward after turn is made
            self.move_forward()
            # setting the first avg wall distance
            prev_wall_avg_side_distance = self.get_side_wall_distance(bypass_opp_dir)
            # variable to prevent turtlebot from exiting function even before it starts moving
            moved_off = False

            # Loop to continuously check till turtlebot is done
            # Breaks out of loop automatically when done
            while (True):

                # to update the laser_range values
                rclpy.spin_once(self)
                print("initial")
                print(initialX)
                print("current")
                print(self.Xpos)
                # check if the turtlebot has reached the other side after moving off, with an allowance of 1 cm
                if (moved_off and (abs(initialX - self.Xpos) <= 0.15)):
                    # turtlebot has successfully navigated around the obstacle, exiting function to resume normal navigation
                    self.get_logger().info('Turtlebot successfully navigated around the obstacle')
                    self.rotatebot(bypass_direction)
                    break
                    self.move_forward()

                # check if there is obstacle in front blocking the way
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    front_wall = (self.laser_range[front_angles]<float(stop_distance)).nonzero()

                    if(len(front_wall[0])>0):
                        # stop moving
                        self.stopbot()
                        self.get_logger().info('Obstacle encountered infront')

                        # exit bypass function if edge of maze is reached
                        if (abs(initialY - self.Ypos) >= len(self.mazelayout) * self.resolution - 2):
                            self.get_logger().info('Reached the edge of the maze while bypassing obs')
                            return
                        if np.take(self.laser_range, 90) > np.take(self.laser_range, 270):
                            self.rotatebot(90)
                        else:
                            self.rotatebot(-90)

                        self.move_forward()

                # calculate the current avg distance from wall from front and side
                # self.get_logger().info('Updating current average wall distance')
                curr_wall_avg_side_distance = self.get_side_wall_distance(bypass_opp_dir)
                if (curr_wall_avg_side_distance < prev_wall_avg_side_distance):
                        # reset the the average distance of the wall on the wall
                        prev_wall_avg_side_distance = curr_wall_avg_side_distance
                        self.get_logger().info('New previous average wall distance is %.2f' % prev_wall_avg_side_distance)
                # check if the obstacle is still on the side
                # if distance suddenly increases significantly, obstacle no longer on the side
                # checked by if the avg distance between previous and current distance from wall defer by more than 50%
                # and the prev_wall_avg_distance needs to be less than 0.5m away to ensure the wall has been detected before
                distance_diff = curr_wall_avg_side_distance - prev_wall_avg_side_distance
                if ((distance_diff > 2 * prev_wall_avg_side_distance) and (prev_wall_avg_side_distance <= 0.5)):
                    self.get_logger().info('Side wall no longer detected')
                    self.get_logger().info('Distance diff is this %.2f' % distance_diff)
                    # extra distance so that the turtlebot has sufficient space to turn
                    timenow = time.time()
                    while time.time() - timenow < 0.8:
                        print("stopping")
                    self.stopbot()
                    # wall no longer detected, stop and rotate turtlebot   
                    self.rotatebot(bypass_opp_dir)
                    # set moved_off to True only once
                    if (moved_off == False):
                        moved_off = True
                        self.get_logger().info('Moved_off set to True')

                    # reset the avg side wall distance
                    self.get_logger().info('Resetting prev average side wall distance')
                    prev_wall_avg_side_distance = self.get_side_wall_distance(bypass_opp_dir)
                    self.get_logger().info('New prev average wall distance is %.2f' % prev_wall_avg_side_distance)
                    self.move_forward()
        

        

    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            while (self.laser_range.size == 0):
                print("Spin to get a valid lidar data")
                print(self.laser_range)
                rclpy.spin_once(self)
            self.timenow = time.time()
            while rclpy.ok():
                # print(time.time())
                print("target")
                print(self.Xstart)
                print(self.Ystart)
                print("current")
                print(self.Xpos)
                print(self.Ypos)
                if time.time() - self.timenow > initialdelay and self.recordedinitial == False:
                    self.recordedinitial = True
                    self.recordposition()
                if self.moved_off == False and (abs(self.Xstart - self.Xpos) > 0.25 or abs(self.Ystart - self.Ypos) > 0.25) and self.recordedinitial:
                    print("################")
                    print("################")
                    print("################")
                    print("################")
                    print("################")
                    print("moved off")
                    self.moved_off = True
                if self.moved_off and self.loaded and (abs(self.Xstart - self.Xpos) < 0.5 and abs(self.Ystart - self.Ypos) < 0.5) and self.one_round == False:
                    print("################")
                    print("################")
                    print("################")
                    print("################")
                    print("################")
                    print("moved one round")
                    self.one_round = True

                if self.loaded == False and self.nfc_presence == True:
                  self.loading()
                if self.loaded and self.one_round and self.target_presence:
                #   self.cut_through()
                  self.stopbot()
                  print("IM DONE, TIME FOR ##################################################################################")
                  break
                self.pick_direction()
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    targeter = Targeter()
    rclpy.spin(targeter)
    targeter.destroy_node()
    #do some thermal nav node and shooting node here
    rclpy.shutdown()


if __name__ == '__main__':
    main()
