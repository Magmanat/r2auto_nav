import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
import numpy as np
import time
import gpiozero as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory



class Servonode(Node):
    
    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            Bool,
            '/servo_on',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        factory = PiGPIOFactory()
        # Set pin numbering convention
        # GPIO.setmode(GPIO.BOARD)
        # Choose an appropriate pwm channel to be used to control the servo
        self.servo_pin = 18
        # Set the pin as an output

        # Initialise the servo to be controlled by pwm with 50 Hz frequency
        # p = GPIO.PWM(servo_pin, 50)
        # Set servo to 15 degrees as it's starting position
        self.startangle = 0
        self.endangle = 50
        self.sweepingTime = 1
        self.s = GPIO.AngularServo(self.servo_pin, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0024, pin_factory=factory)

        # p.start(DegreesToDutyCycle(startangle))
        

    def listener_callback(self, msg):
        if msg.data == True:
          self.s.angle = self.endangle
        else:
          self.s.angle = self.startangle
        


def main(args=None):
    rclpy.init(args=args)

    servonode = Servonode()
    try:
      rclpy.spin(servonode)
    except KeyboardInterrupt:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
      servonode.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
