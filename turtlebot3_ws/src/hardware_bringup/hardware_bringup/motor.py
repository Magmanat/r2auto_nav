import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
import numpy as np
import time
import gpiozero as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory



class Motornode(Node):
    
    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            Bool,
            '/motor_on',
            self.listener_callback,
            10)

        self.motorpin1 = 20
        self.motorpin2 = 21
        # Set the pin as an output
        self.motor1 = GPIO.LED(self.motorpin1)
        self.motor2 = GPIO.LED(self.motorpin2)

        

    def listener_callback(self, msg):
        if msg.data == True:
          self.motor1.on()
          self.motor2.on()
        else:
          self.motor1.off()
          self.motor2.off()
        


def main(args=None):
    rclpy.init(args=args)

    motornode = Motornode()
    try:
      rclpy.spin(motornode)
    except KeyboardInterrupt:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
      motornode.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()