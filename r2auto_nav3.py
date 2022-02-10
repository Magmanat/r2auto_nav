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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from time import sleep
import matplotlib.pyplot as plt
from PIL import Image
import scipy.stats
import numpy as np
import math
import cmath
import time
from rclpy.duration import Duration

from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 

# constants
rotatechange = 0.2
speedchange = 0.25
occ_bins = [-1, 0, 50, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

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

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        
        self.declare_parameter('target_frame', 'base_footprint')
        self.target_frame = self.get_parameter(
        'target_frame').get_parameter_value().string_value
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # create subscription to track lidar
        # self.scan_subscription = self.create_subscription(
        #     LaserScan,
        #     'scan',
        #     self.scan_callback,
        #     qos_profile_sensor_data)
        # self.scan_subscription  # prevent unused variable warning
        # self.laser_range = np.array([])

        # self.nfcsubscription = self.create_subscription(Bool,
        # 'NFC_presence',
        # self.NFC_callback,
        # 50)
        self.phase1 = True
        self.phase2 = False
        self.phase3 = False
        self.phase4 = False
        self.Xpos = 0
        self.Ypos = 0
        self.XposNoAdjust = 0
        self.YposNoAdjust = 0
        self.mazelayout = []
        # self.visitedarraynoadjust = []
        self.visitedarray = np.zeros((300,300),int)
        self.previousaction = []
        self.resolution = 0.05
        self.Xadjust = 0
        self.Yadjust = 0
        self.direction = ''

    # def NFC_callback(self,msg):
    #     if msg.data == True:
    #         self.phase1 = False
    #         self.phase2 = True
    #         self.nfcsubscription.reset()

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)



    def occ_callback(self, msg):
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

        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
    
        trans = None
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now,
                        timeout = Duration(seconds=1))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        self.Xpos = int(np.rint((trans.transform.translation.x - msg.info.origin.position.x)/self.resolution))
        self.Ypos = int(np.rint((trans.transform.translation.y - msg.info.origin.position.y)/self.resolution))
        self.mazelayout[self.Ypos][self.Xpos] = 6
        self.XposNoAdjust = int(np.rint((trans.transform.translation.x + 5)/self.resolution))
        self.YposNoAdjust = int(np.rint((trans.transform.translation.y + 5)/self.resolution))
        self.Xadjust = msg.info.origin.position.x
        self.Yadjust = msg.info.origin.position.y

        
        img2 = Image.fromarray(self.mazelayout)
        img = Image.fromarray(np.uint8(self.visitedarray.reshape(300,300)))
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        # # pause to make sure the plot gets created
        plt.pause(0.00000000001)


    # def scan_callback(self, msg):
    #     # self.get_logger().info('In scan_callback')
    #     # create numpy array
    #     self.laser_range = np.array(msg.ranges)
    #     # print to file
    #     # np.savetxt(scanfile, self.laser_range)
    #     # replace 0's with nan
    #     self.laser_range[self.laser_range==0] = np.nan


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
        target_yaw = math.radians(rot_angle)
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


    def robotforward(self):
        
        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.publisher_.publish(twist)
        twist.linear.x = 0.0
        time.sleep(0.5)
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def move(self, direction):
      # self.visitedarraynoadjust.append([self.XposNoAdjust,self.YposNoAdjust])
      self.visitedarray[self.YposNoAdjust][self.XposNoAdjust] = 1
      if direction == 'down':
        # if self.direction != 'down':
        self.rotatebot(180)
        print("going down")
        self.robotforward()
        self.direction = 'down'
      if direction == 'right':
        # if self.direction != 'right':
        self.rotatebot(-90)
        print("going right")
        self.robotforward()
        self.direction = 'right'
      if direction == 'up':
        # if self.direction != 'up':
        self.rotatebot(0)
        print("going up")
        self.robotforward()
        self.direction = 'up'
      if direction == 'left':
        # if self.direction != 'left':
        self.rotatebot(90)
        print("going left")
        self.robotforward()
        self.direction = 'left'

    def is_empty(self, direction):
      square = [[-2,2],[-1,2],[0,2],[1,2],[2,2],[-2,1],[-1,1],[0,1],[1,1],[2,1],[-2,0],[-1,0],[0,0],[1,0],[2,0],[-2,-1],[-1,-1],[0,-1],[1,-1],[2,-1],[-2,-2],[-1,-2],[0,-2],[1,-2],[2,-2]]
      if direction == 'right':
        for i in square:
          if self.mazelayout[self.Ypos - 4 + i[1]][self.Xpos + i[0]] == 3:
            print("right obstacle")
            return False
      elif direction == 'up':
        for i in square:
          if self.mazelayout[self.Ypos + i[1]][self.Xpos + 4 + i[0]] == 3:
            print("up obstacle")
            return False
      elif direction == 'left':
        for i in square:
          if self.mazelayout[self.Ypos + 4 + i[1]][self.Xpos + i[0]] == 3:
            print("left obstacle")
            return False
      elif direction == 'down':
        for i in square:
          if self.mazelayout[self.Ypos+ i[1]][self.Xpos - 4 + i[0]] == 3:
            print("down obstacle")
            return False
      print(direction + " not obstacle")
      return True

    def is_visited(self, direction):
      square = [[-2,2],[-1,2],[0,2],[1,2],[2,2],[-2,1],[-1,1],[0,1],[1,1],[2,1],[-2,0],[-1,0],[0,0],[1,0],[2,0],[-2,-1],[-1,-1],[0,-1],[1,-1],[2,-1],[-2,-2],[-1,-2],[0,-2],[1,-2],[2,-2]]
      # for i in self.visitedarraynoadjust:
      #   self.visitedarray.append([int(np.rint(((i[0] - self.Xadjust/self.resolution)))),int(np.rint(((i[1] - self.Yadjust)/self.resolution)))])
      print("in visited")
      print("X pos: " + str(self.Xpos))
      print("Y pos: " + str(self.Ypos))
      print("X pos no adj: " + str(self.XposNoAdjust))
      print("Y pos no adj: " + str(self.YposNoAdjust))
      # print(self.visitedarray)
      # np.savetxt(mapfile, self.visitedarray)
      if direction == 'right':
        for i in square:
          print(self.visitedarray[self.YposNoAdjust - 4 + i[1]][self.XposNoAdjust + i[0]])
          if self.visitedarray[self.YposNoAdjust - 4 + i[1]][self.XposNoAdjust + i[0]] == 1:
            print("right visited")
            return True
      elif direction == 'up':
        for i in square:
          print(self.visitedarray[self.YposNoAdjust + i[1]][self.XposNoAdjust + 4 + i[0]])
          if self.visitedarray[self.YposNoAdjust + i[1]][self.XposNoAdjust + 4 + i[0]] == 1:
            print("up visited")
            return True
      elif direction == 'left':
        for i in square:
          print(self.visitedarray[self.YposNoAdjust + 4 + i[1]][self.XposNoAdjust + i[0]])
          if self.visitedarray[self.YposNoAdjust + 4 + i[1]][self.XposNoAdjust + i[0]] == 1:
            print("left visited")
            return True
      elif direction == 'down':
        for i in square:
          print(self.visitedarray[self.YposNoAdjust+ i[1]][self.XposNoAdjust - 4 + i[0]])
          if self.visitedarray[self.YposNoAdjust+ i[1]][self.XposNoAdjust - 4 + i[0]] == 1:
            print("down visited")
            return True
      print(direction + " not visited")
      return False

    def backtrack(self):
        print("backtracking now")
        self.move(self.previousaction[-1])
        self.previousaction.pop(-1)


    def iterative_maze(self):
        directions = [['up','down'],['right','left'],['down','up'],['left','right']]
        while self.phase1:
            needbacktrack = False
            for i in range(len(directions)):
                # print("in directions")
                for b in range(100):
                        rclpy.spin_once(self)
                if (not self.is_visited(directions[i][0])) and (self.is_empty(directions[i][0])):
                    # print("in recursive maze")
                    self.move(directions[i][0])
                    self.previousaction.append(directions[i][1])
                    break
                if i == len(directions):
                    print("end of for loop")
                    needbacktrack = True
            if needbacktrack == True:
                self.backtrack()
        return


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            for i in range(100):
                rclpy.spin_once(self)
            while rclpy.ok():
              if self.phase1:
                self.iterative_maze()
              if self.phase2:
                self.fill_balls()
              if self.phase3:
                self.find_thermal()
              if self.phase4:
                self.fire_cannon()
              for i in range(50):
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
