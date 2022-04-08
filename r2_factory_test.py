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
import os
import math
import cmath
import time


class FactoryTest(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)

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

        self.firing_pub = self.create_publisher(Bool,'fire',10)

        #for others
        self.nfc_presence = False
        self.button_presence = False
        self.laser_valid = False

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

    def scan_callback(self, msg):
        # create numpy array
        self.laser_range = np.array(msg.ranges)

        self.laser_range[self.laser_range==0] = np.Inf
        numinfs = (self.laser_range==np.Inf).sum()
        if numinfs >= 270:
            self.laser_valid = False
        else:
            self.laser_valid = True

    def stopbot(self):
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def clear(self):
        os.system('clear')

    def dynamixeltest(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        done = False
        while not done:
            self.clear()
            print("Input Y and observe if the turtlebot moves approximately 10cm front, 10cm back\nand then 90 degrees left and 90 degrees right")
            if input() == 'y':
                #forward
                twist.linear.x = 0.2
                self.publisher_.publish(twist)
                time.sleep(0.5)
                self.stopbot()
                time.sleep(0.5)
                #backward
                twist.linear.x = -0.2
                self.publisher_.publish(twist)
                time.sleep(0.5)
                self.stopbot()
                time.sleep(0.5)
                #leftward
                twist.linear.x = 0.0
                twist.angular.z = 1.0
                self.publisher_.publish(twist)
                time.sleep(np.pi/2)
                self.stopbot()
                time.sleep(0.5)
                #rightward
                twist.linear.x = 0.0
                twist.angular.z = -1.0
                self.publisher_.publish(twist)
                time.sleep(np.pi/2)
                self.stopbot()
                time.sleep(0.5)
                done = True

    def lidartest(self):
        done = False
        while not done:
            self.clear()
            print("input Y to start the lidar test")
            if input() == 'y':
                done = True
        while (self.laser_range.size == 0):
                self.clear()
                print("Spin to get a valid lidar data")
                print(self.laser_range)
                rclpy.spin_once(self)
        if self.laser_valid:
            for i in [3,2,1]:
                self.clear()
                print("laserscan data is VALID, lidar is functional,\nproceeding to the nfc test in " + str(i) + " ...")
                time.sleep(1)
        else:
            for i in [3,2,1]:
                self.clear()
                print("laserscan data is INVALID, lidar is not functional,\nproceeding to the nfc test in " + str(i) + " ...")
                time.sleep(1)

    def nfctest(self):
        done = False
        timenow = time.time()
        nfc_test_duration = 20
        while not done:
            self.clear()
            print("place nfc tag below nfc sensor, there is 20 seconds to test the NFC\n" + str(int(time.time() - timenow)))
            rclpy.spin_once(self)
            if self.nfc_presence == True:
                for i in [3,2,1]:
                    self.clear()
                    print("NFC is functional,\nproceeding to the button test in " + str(i) + " ...")
                    time.sleep(1)
                done = True
            if time.time() - timenow > nfc_test_duration:
                for i in [3,2,1]:
                    self.clear()
                    print("NFC is NOT functional or has not been placed at sensor,\nproceeding to the button test in " + str(i) + " ...")
                    time.sleep(1)
                done = True
            
    def buttontest(self):
        done = False
        timenow = time.time()
        button_test_duration = 20
        while not done:
            self.clear()
            print("press the button to test, there is 20 seconds to test the button\n" + str(int(time.time() - timenow)))
            rclpy.spin_once(self)
            if self.button_presence == True:
                for i in [3,2,1]:
                    self.clear()
                    print("button is functional,\n proceeding to the firing test in " + str(i) + " ...")
                    time.sleep(1)
                done = True
            if time.time() - timenow > button_test_duration:
                for i in [3,2,1]:
                    self.clear()
                    print("button is NOT functional or has not been pressed,\nproceeding to the firing test in " + str(i) + " ...")
                    time.sleep(1)
                done = True
    def firingtest(self):
        fire = Bool()
        fire.data = True
        done = False
        while not done:
            self.clear()
            print("Place the turtlebot in front of you with the firing tube facing towards yourself\ninput Y and observe that the left motor spins, followed by the right motor\nfollowed by the servo opening, and finally the servo closes and motors shut down")
            if input() == 'y':
                #forward
                self.firing_pub.publish(fire)
                time.sleep(6)
                done = True

    def thermaltest(self):
            #for thermal stuff
            self.thermal_array = [0 for i in range(64)]
            #plot start
            plt.rcParams.update({'font.size':16})
            self.fig_dims = (12,9) # figure size
            self.fig,self.ax = plt.subplots(figsize=self.fig_dims) # start figure
            self.pix_res = (8,8) # pixel resolution
            self.zz = np.zeros(self.pix_res) # set array with zeros first
            self.im1 = self.ax.imshow(self.zz,vmin=25,vmax=35) # plot image, with temperature bounds
            self.cbar = self.fig.colorbar(self.im1,fraction=0.0475,pad=0.03) # colorbar
            self.cbar.set_label('Temperature [C]',labelpad=10) # temp. label
            self.fig.canvas.draw() # draw figure
            self.ax_bgnd = self.fig.canvas.copy_from_bbox(self.ax.bbox) # background for speeding up runs
            self.fig.show() # show figure 

            def thermal_callback(thermal_array):
                pix_res = (8,8)
                self.thermal_array = np.reshape(thermal_array.data,pix_res)
                #plot update
                self.fig.canvas.restore_region(self.ax_bgnd) # restore background (speeds up run)
                self.im1.set_data(np.reshape(self.thermal_array,pix_res)) # update plot with new temps
                self.ax.draw_artist(self.im1) # draw image again
                self.fig.canvas.blit(self.ax.bbox) # blitting - for speeding up run

            self.fig.canvas.flush_events() # for real-time plot
            self.subscription = self.create_subscription(
            Float64MultiArray,
            'thermal',
            thermal_callback,
            10)
            timenow = time.time()
            thermal_test_duration = 20
            done = False
            while not done:
                self.clear()
                print("Check if the thermal camera can pick up the heat from your hand, you have 20 seconds to check this\n" + str(int(time.time() - timenow)))
                rclpy.spin_once(self)
                if time.time() - timenow > thermal_test_duration:
                    done = True


    def test(self):
        try:
            self.dynamixeltest()
            self.lidartest()
            self.nfctest()
            self.buttontest()
            self.firingtest()
            self.thermaltest()
                
        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)
    factorytest = FactoryTest()
    factorytest.test()
    factorytest.destroy_node()
    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
        #do some thermal nav node and shooting node here
    rclpy.shutdown()


if __name__ == '__main__':
    main()