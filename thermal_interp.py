import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy import interpolate

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
        # original resolution
        self.pix_res = (8,8) # pixel resolution
        self.xx,self.yy = (np.linspace(0,self.pix_res[0],self.pix_res[0]),
                            np.linspace(0,self.pix_res[1],self.pix_res[1]))
        zz = np.zeros(self.pix_res) # set array with zeros first
        # new resolution
        pix_mult = 6 # multiplier for interpolation 
        interp_res = (int(pix_mult*self.pix_res[0]),int(pix_mult*self.pix_res[1]))
        self.grid_x,self.grid_y = (np.linspace(0,self.pix_res[0],interp_res[0]),
                                    np.linspace(0,self.pix_res[1],interp_res[1]))


        self.grid_z = self.interp(zz) # interpolated image

        #plot start
        plt.rcParams.update({'font.size':16})
        self.fig_dims = (12,9) # figure size
        self.fig,self.ax = plt.subplots(figsize=self.fig_dims) # start figure
        self.im1 = self.ax.imshow(self.grid_z,vmin=25,vmax=35) # plot image, with temperature bounds
        self.cbar = self.fig.colorbar(self.im1,fraction=0.0475,pad=0.03) # colorbar
        self.cbar.set_label('Temperature [C]',labelpad=10) # temp. label
        self.fig.canvas.draw() # draw figure

        self.ax_bgnd = self.fig.canvas.copy_from_bbox(self.ax.bbox) # background for speeding up runs
        self.fig.show() # show figure    

    #interpolate func
    def interp(self,z_var):
        # cubic interpolation on the image
        # at a resolution of (pix_mult*8 x pix_mult*8)
        
        f = interpolate.interp2d(self.xx,self.yy,z_var,kind='cubic')
        return f(self.grid_x,self.grid_y)

    #function to  store array data in self.thermal_array
    def thermal_callback(self, thermal_array):
        pix_res = (8,8)
        self.get_logger().info('I heard: "%s"' % thermal_array.data)
        self.thermal_array = np.reshape(thermal_array.data,pix_res)
        print(self.thermal_array)

        #plot update
        self.fig.canvas.restore_region(self.ax_bgnd) # restore background (speeds up run)
        self.im1.set_data(self.interp(np.reshape(self.thermal_array,pix_res))) # update plot with new temps
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
    #     pix_to_read = 64 # read all 64 pixels
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