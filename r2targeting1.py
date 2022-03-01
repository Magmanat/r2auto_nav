import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np
import time

#todo 
#Implement front distance
#Implement firing sequence
#integrate Prince's Navigation
#Test with bot

#Please run "sudo chmod a+rw /dev/i2c-1" on bash console if permission denied error on thermal_publisher

''' Subcribe to Thermal Cam Data (array) and publish to twist topic for rotation to centralise
and forward'''

#variable
speedchange = 0.05
rotatechange = 0.1
hot_threshold = 32.0
shoot_distance = 30.0



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
        self.subscription  # prevent unused variable warning

        #some data
        self.thermal_array = [0 for i in range(64)]
        self.target_presence = False
        self.front_distance = 100 #-1 REAL VALUE
        self.centered = False


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
            if self.front_distance > shoot_distance and self.centered == True:
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

    #funtion that moves bot forward by 1 step according speedchange variable and sleep time
    def robotforward(self):
        # start moving
        self.get_logger().info('1 step forward')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.publisher_.publish(twist)
        twist.linear.x = 0.0
        time.sleep(1.5)
        self.publisher_.publish(twist)


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
        if max_value >= hot_threshold:     
            self.target_presence = True
        else:
            self.target_presence = False

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
                twist.angular.z = rotatechange
                time.sleep(1)
                self.publisher_.publish(twist)
                twist.angular.z = 0.0
                time.sleep(1.5)
                self.publisher_.publish(twist)
                self.centered = False
        elif max_col > 4:
                # turn 1 step clockwise
                print("Not centered, 1 step CW")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = -rotatechange
                time.sleep(1)
                self.publisher_.publish(twist)
                twist.angular.z = 0.0
                time.sleep(1.5)
                self.publisher_.publish(twist)
                self.centered = False
        else:
            print("Centered and stopped")
            self.stopbot
            self.centered = True
                    
def main(args=None):
    rclpy.init(args=args)

    targeter = Targeter()
    #input targeter node functions
    rclpy.spin(targeter)
    

    targeter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
