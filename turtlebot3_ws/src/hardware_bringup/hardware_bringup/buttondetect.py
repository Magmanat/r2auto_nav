import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import gpiozero as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory




#publisher to publish if button press is detected or not
class Button_publisher(Node):
    
    def __init__(self):
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(Bool, 'button_pressed', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.buttonpin = 26
        self.b = GPIO.Button(self.buttonpin)

        

    def timer_callback(self):
      msg = Bool()
      msg.data = False
      if self.b.value == 1:
        msg.data = True
      print(msg)
      self.publisher_.publish(msg)
      

def main(args=None):
    rclpy.init(args=args)

    button_publisher = Button_publisher()
    try:
      rclpy.spin(button_publisher)
    except KeyboardInterrupt:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
      button_publisher.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
