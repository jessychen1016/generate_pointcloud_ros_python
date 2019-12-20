#!/usr/bin/python
import sys
import os
import numpy
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from generate_points.msg import image_with_class
from generate_points.msg import position_3d
import cv2



def get_points_data(class_name, total_x, total_y, total_z):

    number_of_object = len(class_name)
    x_string = {}
    y_string = {}
    z_string = {}
    x_float_ = []
    y_float_ = []
    z_float_ = []
    x_float_list = []
    y_float_list = []
    z_float_list = []
    number_of_point_in_class = {}

    print("numbe of class == ", number_of_object)

    for i in range(number_of_object):
        x_string[i] = total_x[i]
        y_string[i] = total_y[i]
        z_string[i] = total_z[i]
        number_of_point_in_class[i] = len(x_string[i].split())

    for i in range(number_of_object):
        for j in range(number_of_point_in_class[i]):
            x_float_.append(float(x_string[i].split()[j]))
            y_float_.append(float(y_string[i].split()[j]))
            z_float_.append(float(z_string[i].split()[j]))
        x_float_list.append(x_float_)
        y_float_list.append(y_float_)
        z_float_list.append(z_float_)
        x_float_ = []
        y_float_ = []
        z_float_ = []
        
    return x_float_list, y_float_list, z_float_list
    



def input_callback(geometry_data):
    global class_name, total_x, total_y, total_z
    class_name = geometry_data.class_name_of_the_box
    total_x = geometry_data.x_positions_of_all_class
    total_y = geometry_data.y_positions_of_all_class
    total_z = geometry_data.z_positions_of_all_class 



if __name__ == '__main__':
    rospy.init_node('Graph_Generator', anonymous=True)
    rospy.Subscriber('/Geometry_Data_of_Detection', position_3d, input_callback) # BGR, Depth, Class(labels and their location)
    graph_pub = rospy.Publisher("Graph_of_Detection", position_3d, queue_size=1)
    rospy.sleep(2)
    while True:
        # pointcloud = generate_pointcloud(rgb_message, depth_message)
        if class_name: # to make sure the program jump into graph_generate only when get new message arrive
            x_float_list, y_float_list, z_float_list = get_points_data(class_name, total_x, total_y, total_z)
            class_name = None # to clear the existed class
