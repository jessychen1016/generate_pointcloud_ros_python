#!/usr/bin/python
import sys
import os
import numpy as np
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from generate_points.msg import image_with_class
from generate_points.msg import position_3d
from generate_points.msg import position_extracted
import cv2
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2



bridge = CvBridge()



def get_points_data(input_header, class_name, total_x, total_y, total_z):

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
    print("z list[0][0] == %.2f"%z_float_list[0][0])
    print("z list[0][1] == %.2f"%z_float_list[0][1])
    return input_header, class_name, x_float_list, y_float_list, z_float_list, number_of_object

def calculate_center(x_float_list, y_float_list, z_float_list, number_of_object):
    x_float_list_np = np.array(x_float_list)
    y_float_list_np = np.array(y_float_list)
    z_float_list_np = np.array(z_float_list)
    for i in range(number_of_object):
        x_float_list_np[i] = np.mean(x_float_list_np[i])
        y_float_list_np[i] = np.mean(y_float_list_np[i])
        z_float_list_np[i] = np.mean(z_float_list_np[i])
    x_float_list = x_float_list_np.tolist()
    y_float_list = y_float_list_np.tolist()
    z_float_list = z_float_list_np.tolist()

    print("x_float_list", x_float_list)
    print("y_float_list", y_float_list)
    print("z_float_list", z_float_list)

    return x_float_list, y_float_list, z_float_list

def draw_geometry_points(x_center_list, y_center_list, z_center_list, number_of_object):
    figure_3d = plt.figure()
    ax = figure_3d.add_subplot(111, projection='3d')   
    for i in range(number_of_object):
        xs = x_center_list[i]
        ys = y_center_list[i]
        zs = z_center_list[i]
        ax.scatter(xs, ys, zs, c='r', marker='o')
        # ax.plot(xs, ys, zs, color='g')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.ion()
    plt.show()
    plt.pause(0.001)
    

    
def input_callback(geometry_data):
    global class_name, total_x, total_y, total_z, input_header
    input_header = geometry_data.header
    class_name = geometry_data.class_name_of_the_box
    total_x = geometry_data.x_positions_of_all_class
    total_y = geometry_data.y_positions_of_all_class
    total_z = geometry_data.z_positions_of_all_class 

def reference_callback(data):
    global referen_image
    referen_image = data.ColorImage

def pt_reference_callback(data):
    global reference_pt
    reference_pt = data


def publish_extracted_geo(output_header, class_name, x_center_list, y_center_list, z_center_list):
    extracted_geo = position_extracted()
    extracted_geo.header = output_header
    extracted_geo.class_name_of_the_box = class_name
    extracted_geo.x_positions = x_center_list
    extracted_geo.y_positions = y_center_list
    extracted_geo.z_positions = z_center_list
    extracted_geo_pub.publish(extracted_geo)



if __name__ == '__main__':
    rospy.init_node('Graph_Generator', anonymous=True)
    rospy.Subscriber('/Geometry_Data_of_Detection', position_3d, input_callback) # BGR, Depth, Class(labels and their location)
    rospy.Subscriber('/class_image_YOLO', image_with_class, reference_callback) # BGR, Depth, Class(labels and their location)
    rospy.Subscriber('/point_cloud2', PointCloud2, pt_reference_callback) # PointCloud2
    global extracted_geo_pub
    extracted_geo_pub = rospy.Publisher("Extracted_Geometry", position_extracted, queue_size=1)

    # graph_pub = rospy.Publisher("Graph_of_Detection", position_3d, queue_size=1)

    global img_reference_pub, pt_reference_pub
    # reference_pub = rospy.Publisher("IMG_Reference_Frame", Image, queue_size=1)
    pt_reference_pub = rospy.Publisher("PT_Reference_Frame", PointCloud2, queue_size=1)
    rospy.sleep(2)
    while True:
        # pointcloud = generate_pointcloud(rgb_message, depth_message)
        if class_name: # to make sure the program jump into graph_generate only when get new message arrive
            # #  to show the reference image of the image and the geometry data (Used for debugging and making sure the position is correction)
            # pt_reference_pub.publish(reference_pt)
            input_header_archieve, class_name_archieve, x_float_list, y_float_list, z_float_list, number_of_object= get_points_data(input_header, class_name, total_x, total_y, total_z)
            x_center_list, y_center_list, z_center_list = calculate_center(x_float_list, y_float_list, z_float_list, number_of_object)
            publish_extracted_geo(input_header_archieve, class_name_archieve, x_center_list, y_center_list, z_center_list)
            class_name = None # to clear the existed class

            # # to show the reference image of the image and the geometry data (Used for debugging and making sure the position is correction)
            # referen_image_mat = bridge.imgmsg_to_cv2(referen_image, "rgb8")
            # cv2.waitKey(3)
            # cv2.imshow("Reference Image", referen_image_mat)
            # cv2.waitKey(3)
            draw_geometry_points(x_center_list, y_center_list, z_center_list, number_of_object) #to draw the point out with matplotlib (could be disabled with commenting it out)
