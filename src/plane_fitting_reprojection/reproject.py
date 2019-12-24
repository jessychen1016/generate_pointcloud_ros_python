#!/usr/bin/python
import sys
import os
import numpy as np
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from generate_points.msg import position_extracted
import cv2
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
bridge = CvBridge()



def plane_fitting(x_center_list, y_center_list, z_center_list):
    figure_3d = plt.figure()
    ax = figure_3d.add_subplot(111, projection='3d')   
    ax.scatter(x_center_list, y_center_list, z_center_list, c='r', marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_autoscaley_on(False)
    ax.set_xlim([-10,10])
    ax.set_ylim([-10,10])
    ax.set_zlim([-10,10])

    # do fit
    tmp_A = []
    tmp_B = []
    for i in range(len(x_center_list)):
        tmp_A.append([x_center_list[i], y_center_list[i], 1])
        tmp_B.append(z_center_list[i])
    b = np.matrix(tmp_B).T
    A = np.matrix(tmp_A)
    fit = (A.T * A).I * A.T * b
    errors = b - A * fit
    residual = np.linalg.norm(errors)
    print "solution:"
    print "%f x + %f y + %f = z" % (fit[0], fit[1], fit[2])
    print "errors:"
    print errors
    print "residual:"
    print residual

    X,Y = np.meshgrid(np.arange(-10, 10), np.arange(-10, 10))
    Z = np.zeros(X.shape)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
    ax.plot_wireframe(X,Y,Z, color='k')


    # plt.ion() # comment this out to stop the program from running and stuck at the for msg recieved
    plt.show()
    plt.pause(0.001)
    

    
def input_callback(extracted_geometry):
    global class_name,  input_header, x_list, y_list, z_list
    input_header = extracted_geometry.header
    class_name = extracted_geometry.class_name_of_the_box
    x_list = extracted_geometry.x_positions
    y_list = extracted_geometry.y_positions
    z_list = extracted_geometry.z_positions



if __name__ == '__main__':
    rospy.init_node('Plane_Reprojection', anonymous=True)
    rospy.Subscriber('/Extracted_Geometry', position_extracted, input_callback) # PointCloud2
    rospy.sleep(2)
    class_name = 0
    while True:
        # pointcloud = generate_pointcloud(rgb_message, depth_message)
        if class_name: # to make sure the program jump into graph_generate only when get new message arrive
            plane_fitting(x_list,y_list,z_list)
            class_name = None # to clear the existed class
            # cv2.waitKey(30000000)

