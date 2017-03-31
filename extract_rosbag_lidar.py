#!/usr/bin/python
"""Extract lidar frontview images from a rosbag.
"""

import os
import sys
import argparse
import numpy as np
import rosbag
import sensor_msgs.point_cloud2
import matplotlib.pyplot as plt
#from PIL import Image
import math

def lidar_to_2d_front_view(points,
                           v_res,
                           h_res,
                           v_fov,
                           values,                            
                           output,
                           cmap = 'jet',
                           y_fudge=0.0,
                           dpi=100
                           ):
    """ Takes points in 3D space from LIDAR data and projects them to a 2D
        "front view" images with  up to three channels (Height, Distance, Intensity) , and saves that images.

    Args:
        points: (np array)
            The numpy array containing the lidar points.
            The shape should be Nx4
            - Where N is the number of points, and
            - each point is specified by 4 values (x, y, z, intensity)
        v_res: (float)
            vertical resolution of the lidar sensor used.
        h_res: (float)
            horizontal resolution of the lidar sensor used.
        v_fov: (tuple of two floats)
            (minimum_negative_angle, max_positive_angle)
        values: (list)
            What value to use to encode the points that get plotted.
            One or more of {"distance", "height", "intensity"}
        cmap: (str)
            Color map to use to color code the values.
            NOTE: Must be a value accepted by matplotlib's scatter function
            Examples: "jet", "gray"
        output: (list)
            List of output filenames.
            If None, then it just shows the image.
        y_fudge: (float)
            A hacky fudge factor to use if the theoretical calculations of
            vertical range do not match the actual data.

            For a Velodyne HDL 64E, set this value to 5.
       dpi: (int)
    """

    # DUMMY PROOFING
    assert len(v_fov) ==2, "v_fov must be list/tuple of length 2"
    assert v_fov[0] <= 0, "first element in v_fov must be 0 or negative"   

    x_lidar = points[:, 0]
    y_lidar = points[:, 1]
    z_lidar = points[:, 2]
    r_lidar = points[:, 3] # intensity
    # Distance relative to origin when looked from top
    d_lidar = np.sqrt(x_lidar ** 2 + y_lidar ** 2)

    v_fov_total = -v_fov[0] + v_fov[1]

    # Convert to Radians
    v_res_rad = v_res * (np.pi/180)
    h_res_rad = h_res * (np.pi/180)

    # PROJECT INTO IMAGE COORDINATES
    x_img = np.arctan2(-y_lidar, x_lidar)/ h_res_rad
    y_img = np.arctan2(z_lidar, d_lidar)/ v_res_rad

    # SHIFT COORDINATES TO MAKE 0,0 THE MINIMUM
    x_min = -360.0 / h_res / 2  # Theoretical min x value based on sensor specs
    x_img -= x_min              # Shift
    x_max = 360.0 / h_res       # Theoretical max x value after shifting

    y_min = v_fov[0] / v_res    # theoretical min y value based on sensor specs
    y_img -= y_min              # Shift
    y_max = v_fov_total / v_res # Theoretical max x value after shifting

    y_max += y_fudge            # Fudge factor if the calculations based on
                                # spec sheet do not match the range of
                                # angles collected by in the data.   

    fig, ax = plt.subplots(figsize=(x_max/dpi, y_max/dpi), dpi=dpi)
    for i in range(len(values)):  
        # WHAT DATA TO USE TO ENCODE THE VALUE FOR EACH PIXEL
        if values[i] == "intensity":
            pixel_values = r_lidar
        elif values[i] == "height":
            pixel_values = z_lidar
        else:
            pixel_values = d_lidar        
        
        ax.scatter(x_img,y_img, s=1, c=pixel_values, linewidths=0, alpha=1, cmap=cmap)
        ax.set_facecolor((0, 0, 0)) # Set regions with no points to black
        ax.axis('scaled')              # {equal, scaled}
        ax.xaxis.set_visible(False)    # Do not draw axis tick marks
        ax.yaxis.set_visible(False)    # Do not draw axis tick marks
        plt.xlim([0, x_max])   # prevent drawing empty space outside of horizontal FOV
        plt.ylim([0, y_max])   # prevent drawing empty space outside of vertical FOV

        fig.savefig(output[i], dpi=dpi, bbox_inches='tight', pad_inches=0.0)
        plt.cla()
        
    plt.close()
    
def main():
    """Extract velodyne points and project to 2D images from a ROS bag
    """
    parser = argparse.ArgumentParser(description="Extract velodyne points and project to 2D images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")   

    args = parser.parse_args()
    bag_file = args.bag_file
    output_dir = args.output_dir 
    if not os.path.isfile(bag_file):
        print('bag_file ' + bag_file + ' does not exist')
        sys.exit()
        
    if not os.path.isdir(output_dir):
        print('output_dir ' + output_dir + ' does not exist')
        sys.exit()
        
    values = ['height', 'distance', 'intensity']
    
    print("Extract velodyne_points from {} into {}".format(args.bag_file, args.output_dir))
    HRES = 0.35         # horizontal resolution (assuming 20Hz setting)
    VRES = 0.4          # vertical res
    VFOV = (-24.9, 2.0) # Field of view (-ve, +ve) along vertical axis
    Y_FUDGE = 5         # y fudge factor for velodyne HDL 64E
    DPI = 100           # Image resolution 
    
    bag = rosbag.Bag(bag_file, "r")
    count = 0
    for topic, msg, t in bag.read_messages(topics=['/velodyne_points']):
        points = sensor_msgs.point_cloud2.read_points(msg, field_names=['x', 'y', 'z', 'intensity'], skip_nans=False)        
        points = np.array(list(points))
               
        outfiles = [(output_dir + '/{0:06d}_{1}.png').format(count, v) for v in values]
        lidar_to_2d_front_view(points, v_res=VRES, h_res=HRES, v_fov=VFOV, values=values,
                               output=outfiles, y_fudge=Y_FUDGE, dpi=DPI)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()