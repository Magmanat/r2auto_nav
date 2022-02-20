import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import time

''' Subcribe to Thermal Cam Data (array) and publish to twist topic for rotation to centralise
and forward'''

#variable
movespeed = 0.05
turnspeed = 0.1
hot_threshold = 30
shoot_threshold = 35


class Targeter(Node):

    def __init__(self):
        super().__init__('targeter')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        #create subscription to thermal cam array data
        self.subscription = self.create_subscription(
            String,
            'thermal',
            self.thermal_callback,
            10)
        self.subscription  # prevent unused variable warning

    def thermal_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)    

def main(args=None):
    rclpy.init(args=args)

    targeter = Targeter()
    

    targeter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
