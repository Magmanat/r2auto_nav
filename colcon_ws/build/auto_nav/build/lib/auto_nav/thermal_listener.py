import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np
import time
import matplotlib.pyplot as plt
from PIL import Image

class ThermalListener(Node):

    def __init__(self):
        super().__init__('themral_listener')
        #create subscription to thermal cam array data
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'thermal',
            self.thermal_callback,
            10)
        self.subscription  # prevent unused variable warning

        #some data
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

    #function to  store array data in self.thermal_array
    def thermal_callback(self, thermal_array):
        pix_res = (8,8)
        self.get_logger().info('I heard: "%s"' % thermal_array.data)
        self.thermal_array = np.reshape(thermal_array.data,pix_res)
        print(self.thermal_array)

        #plot update
        self.fig.canvas.restore_region(self.ax_bgnd) # restore background (speeds up run)
        self.im1.set_data(np.reshape(self.thermal_array,pix_res)) # update plot with new temps
        self.ax.draw_artist(self.im1) # draw image again
        self.fig.canvas.blit(self.ax.bbox) # blitting - for speeding up run
        self.fig.canvas.flush_events() # for real-time plot
        

    # def plot(self,array):
    #     plt.rcParams.update({'font.size':16})
    #     fig_dims = (12,9) # figure size
    #     fig,ax = plt.subplots(figsize=fig_dims) # start figure
    #     pix_res = (8,8) # pixel resolution
    #     zz = np.zeros(pix_res) # set array with zeros first
    #     im1 = ax.imshow(zz,vmin=15,vmax=40) # plot image, with temperature bounds
    #     cbar = fig.colorbar(im1,fraction=0.0475,pad=0.03) # colorbar
    #     cbar.set_label('Temperature [C]',labelpad=10) # temp. label
    #     fig.canvas.draw() # draw figure

    #     ax_bgnd = fig.canvas.copy_from_bbox(ax.bbox) # background for speeding up runs
    #     fig.show() # show figure
    #     #
    #     #####################################
    #     # Plot AMG8833 temps in real-time
    #     #####################################
    #     #
    #     pix_t_read = 64 # read all 64 pixels
    #     while True:
    #         fig.canvas.restore_region(ax_bgnd) # restore background (speeds up run)
    #         im1.set_data(np.reshape(array,pix_res)) # update plot with new temps
    #         ax.draw_artist(im1) # draw image again
    #         fig.canvas.blit(ax.bbox) # blitting - for speeding up run
    #         fig.canvas.flush_events() # for real-time plot

def main(args=None):
    rclpy.init(args=args)
    thermal_listener=ThermalListener()
    rclpy.spin(thermal_listener)
 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thermal_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()