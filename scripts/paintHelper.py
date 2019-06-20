#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import subprocess
import sys
from geometry_msgs.msg import Point


def translate_float(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def translate_int(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return int(rightMin + (valueScaled * rightSpan))

class LoggerAPI():
    def __init__(self):
        pass

    def log_map_to_image(self, map_data, im_path, im_name, poses):
        '''
        # Params
        # map_data: OccupancyGrid Message as published by map_server
        # im_path:  path to save image as string
        # im_name:  name of image as string
        # poses:    list of poses in map frame, they will be painted in the image
        '''

        map_orig = map_data.info.origin
        self.origin = [map_orig.position.x, map_orig.position.y]
        self.resolution = map_data.info.resolution
        self.img_gray = self.__map_data_to_img(map_data)
        self.img_colored = cv2.cvtColor(self.img_gray, cv2.COLOR_GRAY2RGB)
        self.__paint_poses_in_image(poses)
        self.__safe_img(im_name, im_path)

    def __map_data_to_img(self, map_data):
        height, width = map_data.info.height, map_data.info.width
        map_arr = map_data.data
        max_len = len(map_arr) - 1
        print "Map_size: " +str(max_len)
        print "Map_Height: " +str(height)
        print "Map_Width: " +str(width)
        #rospy.logerr("Map_array: %i  Height: %i  Width: %i",max_len, height,
        #             width)
        image = np.zeros((map_data.info.height, map_data.info.width), np.uint8)

        # TODO
        # use a nested for loop to fill in the image
        # Hint: 
        # you need to translate the OccupancyGrid-Values to image-values
        # OccupancyGrid:
        #   min: 0 = free (white)
        #   max: 100 = occupied (black)
        #   special: -1 = unknown (grey)
        # Image:
        #   min: 0 = black
        #   max: 255 = white
        # Hint: use translate_int function to translate OccupancyGrid-values into image-values
        # Attention: the image is a 2D-Array and map_data is 1D-Array
        # Remember from the 1 exercise how to get the right index of map_data
        # Attention: you need to take care of unknown values (grey is 127 in an image)

        return image

    def __safe_img(self, name, path):
        # TODO
        # use the cv2.imwrite - function with the right parameter to save the image
        # Hint: its 1 line of code



    def __paint_poses_in_image(self, poses):
        for point in poses:
            # TODO
            # get the indices of the point px, py coordinates
            # remember how you did in the other exercises and that the image is 2D!
            # colorate the point in the image
            # Hints: indices must be integer
            #        To use a color fill the cell with RGB-values (1D-Array with 3 cells)


