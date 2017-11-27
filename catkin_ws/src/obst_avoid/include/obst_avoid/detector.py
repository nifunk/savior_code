#!/bin/bash/env python

import argparse
import cv2
import numpy as np
from numpy.linalg import inv
from os.path import basename, expanduser, isfile, join, splitext
import socket

import rospy
from sensor_msgs.msg import CompressedImage

from duckietown_utils import d8_compressed_image_from_cv_image, logger, rgb_from_ros, yaml_load, get_duckiefleet_root
from duckietown_utils import get_base_name, load_camera_intrinsics, load_homography, load_map, rectify
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify

class Detector():
    '''class for detecting obstacles'''
    def __init__(self, robot_name='', map_file=''):
        # Robot name
        self.robot_name = robot_name

        # Load camera calibration parameters
	self.intrinsics = load_camera_intrinsics(robot_name)
	self.H = inv(load_homography(self.robot_name))	
	
    def process_image(self, image):

        # YOUR CODE GOES HERE

	#height, width, depth = image.shape #height=480, width=640
	#RECTIFY THE IMAGE FIRST!!!
 
        jpg_data = image[:,:,::-1]
        

	#before sending: REDEFINE DATA
        return d8_compressed_image_from_cv_image(jpg_data)

    def ground2pixel(self, point):
        '''Transforms point in ground coordinates to point in image
        coordinates using the inverse homography'''
	point_calc=np.zeros((3,1),dtype=np.float)
	point_calc= np.dot(self.H,[[point[0]],[point[1]],[1]])

	pixel_int=[int((point_calc[0])/point_calc[2]),int((point_calc[1])/point_calc[2])]
	#print pixel_float
        return pixel_int

    def just2pixel(self, point):
        '''Draw Lines around picture'''
        return [point[0]*640,point[1]*480]


    def render_segments(self, image):
        for segment in self.map_data["segments"]:	
            pt_x = []
            pt_y = []
            for point in segment["points"]:
                frame, ground_point = self.map_data["points"][point]
                pixel = []
                if frame == 'axle':
                    pixel = self.ground2pixel(ground_point)
                elif frame == 'camera':
                    pixel = ground_point
                elif frame == 'image01':
                    pixel = self.just2pixel(ground_point)
                else:
                    logger.info('Unkown reference frame. Using "axle" frame')
                    pixel = self.ground2pixel(ground_point)
                pt_x.append(pixel[0])
                pt_y.append(pixel[1])
            color = segment["color"]
            image = self.draw_segment(image, pt_x, pt_y, color)
        return image

    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red' : ['rgb', [1, 0, 0]],
            'green' : ['rgb', [0, 1, 0]],
            'blue' : ['rgb', [0, 0, 1]],
            'yellow' : ['rgb', [1, 1, 0]],
            'magenta' : ['rgb', [1, 0 ,1]],
            'cyan' : ['rgb', [0, 1, 1]],
            'white' : ['rgb', [1, 1, 1]],
            'black' : ['rgb', [0, 0, 0]]}
        color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_y[0]),(pt_x[1], pt_y[1]),(b * 255, g* 255, r * 255), 5)
        return image
