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

from std_msgs.msg import Bool

import RPi.GPIO as GPIO

from .submodules import *


class NFCPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Bool, 'NFC_presence', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.pn532 = PN532_I2C(debug=False, reset=17, req=27)
        self.pn532.SAM_configuration()


    def timer_callback(self):
        msg = Bool()
        uid = self.pn532.read_passive_target(timeout=0.5)
        if uid is None:
            msg.data = False
        else: msg.data = True
        self.publisher_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = NFCPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
