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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import cmath
import time

# constants
rotatechange = 0.2
speedchange = 0.3
obstaclespeed = 0.2
uturnforwardspeed = 0.1
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.30
turtlebot_length = 0.40
full_width_length = 5.0 - 2*stop_distance - turtlebot_length
angle_error = (2.0/180) * math.pi
front_angle = 20
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
UP = 0
LEFT = 90
RIGHT = -90
DOWN = 180
turn_tracker = []
directiontostring = {
  UP:"up",
  RIGHT:"right",
  LEFT:"left",
  DOWN:"down"
}
stringtodirection = {
  "up":UP,
  "right":RIGHT,
  "left":LEFT,
  "down":DOWN
}
directions_dict = {
  "up":[LEFT,RIGHT],
  "right":[UP,DOWN],
  "down":[RIGHT,LEFT],
  "left":[DOWN,UP]
}

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

        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0 
        self.x_pos = 0
        self.starting_x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.curr_dir = 'up'
        self.distance_travelled = 0
        self.not_set = True
        self.moving_up = True
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callDOWN,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.map2base_sub = self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callDOWN,
            1)

    def map2base_callDOWN(self, msg):
        # self.get_logger().info('In odom_callDOWN')
        self.x_pos, self.y_pos, self.z_pos = msg.position.x, msg.position.y, msg.position.z
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def scan_callDOWN(self, msg):
        # self.get_logger().info('In scan_callDOWN')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    # to set the initial position so that the distance travelled can be tracked
    def get_initial_x_pos(self):
        self.starting_x_pos = self.x_pos
        self.not_set = False
        self.get_logger().info('Starting horizontal distance position = %.2f' % self.starting_x_pos)

    # for measuring the distance travelled 
    def vertical_distance_travelled(self):
        self.distance_travelled = abs(self.x_pos - self.starting_x_pos)
        # self.get_logger().info('Total horizontal distance travelled = %.2f' % self.distance_travelled)

    # check if the robot is at the end of the maze
    def edge_of_maze_reached(self):
        if self.distance_travelled < full_width_length:
            return False
        # reset the initial position
        self.starting_x_pos = self.x_pos
        # reset value of distance travelled
        self.distance_travelled = 0
        return True

    def move_forward(self):
        # start moving forward
        self.get_logger().info('Moving forward')
        twist = Twist()
        twist.linear.x = obstaclespeed
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.get_logger().info('Current direction %s' % self.curr_dir)
        time.sleep(0.2)
        self.publisher_.publish(twist)

    def travel_distance(self, direction, distance):
        direction = stringtodirection[direction]
        if (LEFT - 10 <= direction <= LEFT + 10)  or (RIGHT - 10 <= direction <= RIGHT + 10):
            curr_pos = self.y_pos
            while (abs(curr_pos - self.y_pos) < distance):
                rclpy.spin_once(self)
                if np.average(self.laser_range[0]) <= stop_distance:
                    self.get_logger().info('Obs in front, unable to allocate more travel distance')
                    break
        else:
            curr_pos = self.x_pos
            while (abs(curr_pos - self.x_pos) < distance):
                rclpy.spin_once(self)
                if np.average(self.laser_range[0]) <= stop_distance:
                    self.get_logger().info('Obs in front, unable to allocate more travel distance')
                    break
                     
        self.stopbot()
        self.get_logger().info('Extra travel distance met')

    # function to bypass the obstacle to continue finding the NFC
    def get_side_wall_distance(self, direction):
      if self.laser_range[stringtodirection[direction]-5:stringtodirection[direction]+5] == []:
        return 5
      return np.average(self.laser_range[stringtodirection[direction]-5:stringtodirection[direction]+5])

    def bypass_obstacle(self):
        # to prevent situation in which turtlebot stuck in starting point of maze which is a dead end
        # if no u-turn done yet
        if len(turn_tracker) == 0:
            if (np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT)):
                bypass_direction = 'left'
                bypass_opp_dir = 'right'
            else:
                bypass_direction = 'right'
                bypass_opp_dir = 'left'
        # check prev u-turn direction
        elif turn_tracker[-1] == 'left':
            bypass_direction = 'right'
            bypass_opp_dir = 'left'
        else:
            bypass_direction = 'left'
            bypass_opp_dir = 'right'
        # variable used to hold the current y position of the turtlebot
        initial_y = self.y_pos
        self.get_logger().info('Initial y position is: %.2f' % initial_y)
        # turn the turtlebot before starting adopted Pledge algorithm
        self.get_logger().info('Turtlebot turned %s to start Pledge Algo' % bypass_direction)
        self.rotate_bot_absolute(self.angle_to_rotate(bypass_direction))
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

            # check if the turtlebot has reached the other side after moving off, with an allowance of 1 cm
            if (moved_off and (abs(initial_y - self.y_pos) <= 0.01)):
                # turtlebot has successfully navigated around the obstacle, exiting function to resume normal navigation
                self.get_logger().info('Turtlebot successfully navigated around the obstacle')
                if self.moving_up:
                  self.rotate_bot_absolute(UP)
                else:
                  self.rotate_bot_absolute(DOWN)
                return

            # check if there is obstacle in front blocking the way
            if self.laser_range.size != 0:
                # check distances in front of TurtleBot and find values less
                # than stop_distance
                front_wall = (self.laser_range[front_angles]<float(stop_distance)).nonzero()

                if(len(front_wall[0])>0):
                    # stop moving
                    self.stopbot()
                    self.get_logger().info('Obstacle encountered infront')
                    # to update the variable self.distance_travelled
                    self.vertical_distance_travelled()

                    # exit bypass function if edge of maze is reached
                    if (self.distance_travelled >= full_width_length):
                        self.get_logger().info('Reached the edge of the maze while bypassing obs')
                        return
                    if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
                        self.rotate_bot_absolute(self.angle_to_rotate('left'))
                    else:
                        self.rotate_bot_absolute(self.angle_to_rotate('right'))

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
                if (abs(self.y_pos - initial_y) < 0.20):
                    self.travel_distance(self.curr_dir, abs(self.y_pos - initial_y))
                else:
                    self.travel_distance(self.curr_dir, 0.20)
                # wall no longer detected, stop and rotate turtlebot
                
                self.rotate_bot_absolute(self.angle_to_rotate(bypass_opp_dir))
                # set moved_off to True only once
                if (moved_off == False):
                    moved_off = True
                    self.get_logger().info('Moved_off set to True')

                # reset the avg side wall distance
                self.get_logger().info('Resetting prev average side wall distance')
                prev_wall_avg_side_distance = self.get_side_wall_distance(bypass_opp_dir)
                self.get_logger().info('New prev average wall distance is %.2f' % prev_wall_avg_side_distance)
                self.move_forward()


    def angle_to_rotate(self, LorR):
      if LorR == 'left':
        index = 0
      else:
        index = 1
      print(directions_dict[self.curr_dir][index])
      return directions_dict[self.curr_dir][index]

    # function to rotate the TurtleBot
    def rotate_bot_absolute(self, rot_angle):
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
            # allow the callDOWN functions to run
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
        self.curr_dir = directiontostring[rot_angle]
        self.publisher_.publish(twist)

    def u_turn_left(self):
        self.get_logger().info('Making a left u-turn')
        self.moving_up = not self.moving_up
        # checks if able to turn left
        if np.take(self.laser_range, LEFT) > stop_distance:
            self.get_logger().info('Left u-turn started')

            # rotate left
            self.rotate_bot_absolute(self.angle_to_rotate('left'))

            # start moving
            self.move_forward()
            self.travel_distance(self.curr_dir, 0.30)
            

            if np.take(self.laser_range, LEFT) > stop_distance:
                self.get_logger().info('Completing left u-turn started')

                # rotate left
                self.rotate_bot_absolute(self.angle_to_rotate('left'))

                # to keep track of current turn
                if (len(turn_tracker)) != 0:
                    self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])
                else:
                    self.get_logger().info('No previous u-turn')

                turn_tracker.append('left')
                self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])

                # reset the x pos
                self.get_initial_x_pos()

                # start moving
                self.move_forward()
            
    def u_turn_right(self):
        self.get_logger().info('Making a right u-turn')
        self.moving_up = not self.moving_up
        # checks if able to turn right
        if np.take(self.laser_range, RIGHT) > stop_distance:
            self.get_logger().info('Right u-turn started')

            # rotate right
            self.rotate_bot_absolute(self.angle_to_rotate('right'))

            # start moving
            self.move_forward()
            self.travel_distance(self.curr_dir, 0.30)
            

            if np.take(self.laser_range, RIGHT) > stop_distance:
                self.get_logger().info('Completing right u-turn started')

                # rotate right
                self.rotate_bot_absolute(self.angle_to_rotate('right'))

                # to keep track of current turn
                if (len(turn_tracker)) != 0:
                    self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])
                else:
                    self.get_logger().info('No previous u-turn')

                turn_tracker.append('right')
                self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])

                # reset the x pos
                self.get_initial_x_pos()

                # start moving
                self.move_forward()

    def u_turn_DOWN(self):
        self.get_logger().info('Making a rotational u-turn')
        self.rotate_bot_absolute(self.angle_to_rotate('left'))
        self.rotate_bot_absolute(self.angle_to_rotate('left'))
        self.get_logger().info('Finsished turning')
        self.moving_up = not self.moving_up
        # to ensure the next turn is correct
        if turn_tracker[-1] == 'left':
            turn_tracker.append('right')
        else:
            turn_tracker.append('left')
        # start moving
        self.move_forward()

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def mover(self):
        num_turns = 0
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.move_forward()
            
            while rclpy.ok():
                self.vertical_distance_travelled()
                if self.not_set:
                        self.get_logger().info('Setting initial distance')
                        self.get_initial_x_pos()
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))
                    # check if the initial position has been set

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        self.get_logger().info('Obstacle encountered infront')
                        if self.edge_of_maze_reached():
                            self.get_logger().info('Reached the edge of the maze')
                            # U-turn the turtlebot in the correct direction
                            # checks if wall is on the right side for the very first turn. If it is, start with left u-turn
                            if (len(turn_tracker) == 0):
                                self.get_logger().info('Checking the first turn')
                                if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
                                    self.u_turn_left()
                                else:
                                    self.u_turn_right()
                            else:
                                self.get_logger().info('Checking subsequent turns')
                                # for checking the subsequent turns
                                if turn_tracker[-1] == 'left' and np.take(self.laser_range, RIGHT) > stop_distance:
                                    self.u_turn_right()
                                elif turn_tracker[-1] == 'right' and np.take(self.laser_range, LEFT) > stop_distance:
                                    self.u_turn_left()
                                else:
                                    self.u_turn_DOWN()
                            num_turns += 1
                            self.get_logger().info('Current num of turns: %d' % num_turns)
                        else:
                            self.bypass_obstacle()
                            self.get_logger().info('Finished bypassing obstacle')

                            # start moving forward after bypassing
                            self.move_forward()
                            

                # allow the callDOWN functions to run
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
