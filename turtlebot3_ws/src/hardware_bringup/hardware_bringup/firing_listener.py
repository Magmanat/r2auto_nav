import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
import numpy as np
import time
import gpiozero as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory
import time



#ros node that will start firing sequence when it subscribes to the topic /fire and it is true
class Firingnode(Node):
    
    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            Bool,
            '/fire',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        factory = PiGPIOFactory()
        # Set pin numbering convention
        # GPIO.setmode(GPIO.BOARD)
        # Choose an appropriate pwm channel to be used to control the servo
        self.servo_pin = 18
        # Set the pin as an output
        self.motorpin1 = 20
        self.motorpin2 = 21
        # Set the pin as an output
        self.motor1 = GPIO.LED(self.motorpin1)
        self.motor2 = GPIO.LED(self.motorpin2)
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
            self.motor1.on()
            time.sleep(1.5)
            self.motor2.on()
            time.sleep(1.5)
            self.s.angle = self.endangle
            time.sleep(2)
            self.s.angle = self.startangle
            self.motor1.off()
            self.motor2.off()

        else:
          self.s.angle = self.startangle
          self.motor1.off()
          self.motor2.off()
        


def main(args=None):
    rclpy.init(args=args)

    firingnode = Firingnode()
    try:
      rclpy.spin(firingnode)
    except KeyboardInterrupt:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
      firingnode.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
