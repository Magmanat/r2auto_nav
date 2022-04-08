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

#variables affecting targeting
TARGETspeedchange = 0.05 #forward speed for targeting
TARGETrotatechange = 0.1 #rotation speed for targeting
TARGET_hotthreshhold = 31 #target temperature
TARGETshoot_distance = 0.45 #distance from center of lidar to front before shooting
TARGET_front_angle = 3 #angle of front to measure distance
TARGET_moveres = 0.2 #time for sleep when moving
TARGET_target_not_detected_delay = 0.1 #delay in seconds
TARGET_target_not_detected_counter = 0 #counter to add up to threshhold
TARGET_target_not_detected_threshhold = 10 #how long you want hot target not detected to restart finding / TARGET_target_not_detected_delay = TARGET_target_not_detected_threshold

# variables affecting navigation
fastspeedchange = 0.18 #0.18
slowspeedchange = 0.10#0.10

turning_speed_wf_fast = 1.0 # Fast turn ideal = 1.0
turning_speed_wf_medium = 0.65 #0.65
turning_speed_wf_slow = 0.4 # Slow turn = 0.4

front_d = 0.45
side_d = 0.45
leftwallfollowing = 1  # 1=left,  -1=right

target_count_threshhold = 5 # how long you want hot_target to be spotted before activating firing / hot_timer_delay = target_count_threshhold
hot_timer_delay = 0.1 # in seconds



occ_bins = [-1, 0, 50, 101] #for occupancy grid
initialdelay = 5 # how long before start point is decided in seconds


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

        self.firing_pub = self.create_publisher(Bool,'fire',10)

        #some data
        self.thermal_array = [0 for i in range(64)]
        self.target_presence = False
        self.front_distance = 100
        self.centered = False
        self.not_detected_delay = TARGET_target_not_detected_delay
        self.not_detected_counter = TARGET_target_not_detected_counter
        self.not_detected_threshold = TARGET_target_not_detected_threshhold
        self.not_detected_time = time.time()


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
        self.thermal_array = np.reshape(thermal_array.data,pix_res)
        self.detect_target()
        if self.target_presence == True:
            self.not_detected_counter = 0
            self.stopbot()
            print("Target Detected, Stop Bot")
            self.center_target()
            if (self.front_distance < TARGETshoot_distance):
                if self.centered == False:
                    self.center_target()
                else:
                    self.stopbot()
                    fire = Bool()
                    fire.data = True
                    self.firing_pub.publish(fire)
                    self.destroy_node(self)
            if self.centered == True and self.front_distance > TARGETshoot_distance:
                    #stop this script and move onto shooting script
                self.robotforward()
        else:
            print("Target Not Detected")
            if time.time() - self.not_detected_time >= self.not_detected_delay:
                self.not_detected_time = time.time()
                self.not_detected_counter += 1
                print(self.not_detected_counter)
            if self.not_detected_counter >= self.not_detected_threshold:
                twist = Twist()
                twist.linear.z = 0.5
                self.publisher_.publish(twist)
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
        self.laser_valid = True

        self.nfcsubscription = self.create_subscription(Bool,
        'NFC_presence',
        self.NFC_callback,
        10)
        self.buttonsubscription = self.create_subscription(Bool,
        'button_pressed',
        self.button_callback,
        10)
        self.resolution = 0.05
        self.nfc_presence = False
        self.button_presence = False
        self.recordedinitial = False
        self.moved_off = False
        self.is_one_round = False
        self.is_loaded = False  
        self.Xstart = 0
        self.Ystart = 0
        self.Xpos = 0
        self.Ypos = 0
        self.mapbase = 0
        self.thermal_array = []
        self.target_presence = False
        self.mazelayout = []

        self.targeter_count = 0
        self.targeter_count_threshhold = target_count_threshhold
        self.thermal_time = time.time()
        self.thermal_targeted_time = 300
        self.target_timer = time.time()
        self.timer_threshhold = hot_timer_delay

        

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
            if time.time() - self.target_timer > self.timer_threshhold:
                self.target_timer = time.time()
                print("added to counter")
                self.targeter_count += 1 
                print(self.targeter_count)
            if self.targeter_count >= self.targeter_count_threshhold:
                self.target_presence = True
        else:
            self.targeter_count = 0
            self.target_presence = False


    def scan_callback(self, msg):
        self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # print('################')
        # print(self.laser_range)
        # print('################')
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.Inf
        numinfs = (self.laser_range==np.Inf).sum()
        if numinfs >= 270:
            self.laser_valid = False
        else:
            self.laser_valid = True


    def pick_direction(self):
        # self.get_logger().info('In pick direction:')
        laser_ranges = self.laser_range.tolist()
        # print(min(laser_ranges[46:90]))
        
        # self.front_dist = min(laser_ranges[0:14] + laser_ranges[346:])
        self.front_dist = laser_ranges[0]
        self.leftfront_dist = min(laser_ranges[15:45])
        self.rightfront_dist = min(laser_ranges[315:345])
        self.left_dist = min(laser_ranges[46:90])

        # self.get_logger().info('Front Distance: %s' % str(self.front_dist))
        # self.get_logger().info('Front Left Distance: %s' % str(self.leftfront_dist))
        # self.get_logger().info('Front Right Distance: %s' % str(self.rightfront_dist))

        # Logic for following the wall
        # >d means no wall detected by that laser beam
        # <d means a wall was detected by that laser beam
        self.front_d = front_d #used to be 0.4
        self.side_d = side_d  # wall distance from the robot. It will follow the left wall and maintain this distance
        # Set turning speeds (to the left) in rad/s

        # These values were determined by trial and error.
        self.turning_speed_wf_fast = turning_speed_wf_fast * leftwallfollowing# Fast turn ideal = 1.0
        self.turning_speed_wf_slow = turning_speed_wf_slow * leftwallfollowing # Slow turn = 0.4
        self.turning_speed_wf_medium = turning_speed_wf_medium * leftwallfollowing
        # Set movement speed
        self.forward_speed = fastspeedchange
        self.forward_speed_slow = slowspeedchange
        # Set up twist message as msg
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # self.publisher_.publish(msg)
        if self.laser_valid == False:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif  ((self.leftfront_dist > self.side_d) and self.front_dist > self.front_d and self.rightfront_dist > self.side_d):
            # print('here')
            if self.left_dist < 0.20:
                # print('wall still here')
                self.wall_following_state = "turn right"
                msg.linear.x = self.forward_speed
                msg.angular.z = -self.turning_speed_wf_slow # turn left to find wall
            else:
                # print('wall just disappeared')
                self.wall_following_state = "search for wall"
                msg.linear.x = self.forward_speed_slow
                msg.angular.z = self.turning_speed_wf_fast # turn left to find wall

        elif self.leftfront_dist > self.side_d and self.front_dist < self.front_d and self.rightfront_dist > self.side_d:
            # print('here2')
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif ((self.leftfront_dist < self.side_d or self.left_dist < self.side_d) and self.front_dist > self.front_d and self.rightfront_dist > self.side_d):
            # print('here3')
            if (self.leftfront_dist < 0.25 or self.left_dist < 0.20):
                # print('left wall close')
                # Getting too close to the wall
                self.wall_following_state = "turn right"
                msg.linear.x = self.forward_speed_slow
                msg.angular.z = -self.turning_speed_wf_medium
            else:
                # Go straight ahead
                # print('left wall far')
                self.wall_following_state = "follow wall"
                msg.linear.x = self.forward_speed


        elif self.leftfront_dist > self.side_d and self.front_dist > self.front_d and self.rightfront_dist < self.side_d:
            # print('here4')
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed_slow
            msg.angular.z = self.turning_speed_wf_slow  # turn left to find wall

        elif self.leftfront_dist < self.side_d and self.front_dist < self.front_d and self.rightfront_dist > self.side_d:
            # print('here5')
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist > self.side_d and self.front_dist < self.front_d and self.rightfront_dist < self.side_d:
            # print('here6')
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist < self.side_d and self.front_dist < self.front_d and self.rightfront_dist < self.side_d:
            # print('here7')
            self.wall_following_state = "turn right"
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist < self.side_d and self.front_dist > self.front_d and self.rightfront_dist < self.side_d:
            # print('here8')
            # Go straight ahead
            # print("left")
            self.wall_following_state = "find wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow

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
        self.is_loaded = True
        
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
                if self.is_one_round == False and ((self.moved_off and (abs(self.Xstart - self.Xpos) < 0.2 and abs(self.Ystart - self.Ypos) < 0.2)) or (time.time() - self.target_timer >= self.thermal_targeted_time)):
                    print("################")
                    print("################")
                    print("################")
                    print("################")
                    print("################")
                    print("moved one round")
                    self.is_one_round = True

                if self.is_loaded == False and self.nfc_presence == True:
                  self.loading()
                if self.is_loaded and self.is_one_round:
                    print("finding thermal now")
                if self.is_loaded and self.is_one_round and self.target_presence:
                #   self.cut_through()
                  self.stopbot()
                  print("IM DONE, TIME FOR FIRING")
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
    targeter = Targeter()
    auto_nav.mover()
    auto_nav.destroy_node()
    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.spin(targeter)
    targeter.destroy_node()
    
        #do some thermal nav node and shooting node here
    rclpy.shutdown()


if __name__ == '__main__':
    main()
