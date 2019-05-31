# -*- coding: utf-8 -*-
"""
Created on Thu May 30 22:38:55 2019

@author: FoolPepper

for pcl in python: pointXYZI to pointXYZRGB

"""

import pcl
import numpy as np
import pcl.pcl_visualization
from struct import pack, unpack

def float_to_rgb(color):
    """ Converts an float value to a RGB list(size 3, which represent the r, g, b of the color channels) in pcl-python
    
        Args:
            color (float): a np.float32 type value
            
        Returns:
            rgb_list: a list, size 3
    """
    rgb_list = np.array([0, 0, 0], dtype = np.float32)
    tmp = pack('f', color)
    a = unpack('i', tmp)[0]      #C++ reinterpret_cast功能
    
    rgb_list[0] = (a >> 16) & 0X0000ff     #r
    rgb_list[1] = (a >> 8) & 0X0000ff      #g
    rgb_list[2] = (a) & 0X0000ff           #b
    
    return rgb_list
    
    
def rgb_to_float(color):
    """ Converts an RGB list to the packed float format used by PCL
    
        From the PCL docs:
        "Due to historical reasons (PCL was first developed as a ROS package),
         the RGB information is packed into an integer and casted to a float"
    
        Args:
            color (list): 3-element list of integers [0-255,0-255,0-255]
            
        Returns:
            float_rgb: RGB value packed as a float
    """
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = unpack('f', pack('i', hex_rgb))[0]

    return float_rgb


def Intensity_to_color(intensity):
    """Converts lidar intensity to the color
    
        Args: 
            intensity(int): the point intensity
        
        Returns:
            color(list): A list (size: 3), represent the r, g, b value 
    
    """
    intensity_map = np.uint32
    intensity_map = intensity
    color = np.array([0, 0, 0], dtype = np.int)
    
    if(intensity_map <= 100):
        if(intensity_map <= 34):
            Green = intensity_map * 255 // 34
            color[0] = 0X00
            color[1] = Green & 0XFF
            color[2] = 0XFF
        
        elif(intensity_map <= 67):
            Blue = (((67 - intensity_map) * 255) // 33)
            color[0] = 0X00
            color[1] = 0XFF
            color[2] = Blue & 0XFF
        else:
            Red = (((intensity_map - 67) * 255) // 33)
            color[0] = Red & 0XFF
            color[1] = 0XFF
            color[2] = 0X00
    else:
        Green = (((255 - intensity_map) * 255) // (256 - 100))
        color[0] = 0XFF
        color[1] = Green & 0XFF
        color[2] = 0X00 
        
    return color     

        
def XYZI_to_XYXRGB(pointXYZI):
    """ Converts an pointXYZI to the pointXYZRGB
    
        Args:
            pointXYZI: a pointCloud which type is pointXYZI
            
        Returns:
            pointXYZRGB
    """
    cloud_rgb = pcl.PointCloud_PointXYZRGB()
    points_list = []
    
    for data in pointXYZI:
        intensity = int(data[3]) 
        print(intensity)
        float_rgb = rgb_to_float(Intensity_to_color(intensity))
        points_list.append([data[0], data[1], data[2], float_rgb])
    
    cloud_rgb.from_list(points_list)
    
    return cloud_rgb

    
if __name__ == "__main__":
    file_name = "frame_save_244.pcd"
    origin = pcl.load_XYZI("C:/Users/Administrator/Desktop/pcds/" + file_name)
    pointXYZRGB = XYZI_to_XYXRGB(origin)
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowColorCloud(pointXYZRGB, b'cloud')
    










