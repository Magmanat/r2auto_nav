import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import gpiozero as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory



class Scanner(Node):
    
    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
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
        self.startangle = 15
        self.endangle = 165
        self.sweepingTime = 1
        self.s = GPIO.AngularServo(self.servo_pin, initial_angle=15, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0024, pin_factory=factory)

        # p.start(DegreesToDutyCycle(startangle))
        

    def listener_callback(self, msg):
        # create array of ranges
        laser_range = msg.ranges
        # remove 0's from dataset
        laser_range = laser_range[-5:] + laser_range[:5]
        new_laser_range = []
        for i in range(len(laser_range)):
          if laser_range[i] == 0:
            continue
          new_laser_range.append(laser_range[i])
        if new_laser_range == []:
          new_laser_range = [-1]
        meanrangecm = np.rint(np.mean(new_laser_range)*100)
        
        self.get_logger().info('mean range is %i cm' % meanrangecm)
        
        if  98 <= meanrangecm <= 100:
          for i in range(100):
            if i % 2 == 1:
              self.s.angle = self.endangle
            else:
              self.s.angle = self.startangle
            time.sleep(0.3)
          rclpy.shutdown()
          # for i in range(self.startangle,self.endangle + 1):
          #   self.s.angle = i
          #   self.get_logger().info('%i'% i)
          #   time.sleep(self.sweepingTime/(self.endangle - self.startangle + 1))
          # self.s.angle = self.startangle

        # log the info
        


def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()
    try:
      rclpy.spin(scanner)
    except KeyboardInterrupt:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
      scanner.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
