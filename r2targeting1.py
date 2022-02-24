import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np
import time

''' Subcribe to Thermal Cam Data (array) and publish to twist topic for rotation to centralise
and forward'''

#variable
speedchange = 0.05
rotatechange = 0.1
hot_threshold = 30
shoot_distance = 30



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
        self.thermal_array = []
        self.target_presence = False
        self.front_distance = -1

#function to  store array data in self.thermal_array
    def thermal_callback(self, thermal_array):
        pix_res = (8,8)
        self.get_logger().info('I heard: "%s"' % thermal_array.data)
        self.thermal_array = np.reshape(thermal_array.data,pix_res)
        print(self.thermal_array)

#function to stop bot
    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist) 

#funtion that moves bot forward by speedchange variable
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
        time.sleep(1.5)
        self.publisher_.publish(twist)


#function that checks through array if there is anything hot
    def detect_target(self):
        while self.target_presence == False:
            for row in range(len(self.thermal_array)):
                for col in range(len(self.thermal_array[row])):
                    if self.thermal_array[row][col] >= hot_threshold:
                        self.target_presence == True
                    else:
                        self.target_presence == False

    def center_target(self):
        max_row = 0
        max_col = 0
        max_value = 0.0
        view = self.thermal_array
        while max_col >4 or max_col<3:
            for row in range(len(self.thermal_array)):
                for col in range(len(self.thermal_array[row])):
                    current_value = view[row][col]
                    if current_value > max_value:
                        max_row = row
                        max_col = col
                        max_value = current_value
            if max_col < 3:
                    # spin it anti-clockwise
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
            elif max_col > 4:
                    # spin it clockwise
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = -1 * rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
            else:
                self.stopbot
                    
def main(args=None):
    rclpy.init(args=args)

    targeter = Targeter()
    #input targeter node functions

    #trying to detect hot target. if detected, stop
    while targeter.target_presence == False:
        targeter.detect_target()
        if targeter.target_presence == True:
            targeter.stopbot()
    while targeter.front_distance < shoot_distance:
        targeter.center_target()
        targeter.robotforward
    
    
    

    targeter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
