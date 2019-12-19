#!/usr/bin/python
import sys
import os
import numpy
import rospy
import roslib
from generate_points.msg import image_with_class
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import Image as rosImage
from generate_points.msg import image_with_class
from generate_points.msg import position_3d
import cv2



bridge = CvBridge()


focal_length = 378.95
cy = 242.83
cx = 321.53
scalingFactor = 4000.00



def generate_pointcloud(rgb_message, depth_message):

    # generate pointclouds from rgb(or mono) and depth

    cv_rgb = bridge.imgmsg_to_cv2(rgb_message, "rgb8")
    cv_depth = bridge.imgmsg_to_cv2(depth_message, "16UC1")
    

    # add some filters

    cv_depth = cv2.medianBlur(cv_depth,7)
    # cv_depth = cv2.blur(cv_depth, (9,9))
    # cv_depth = cv2.GaussianBlur(cv_depth, (3,3), 1)
    


    # if rgb.size != depth.size:
    #     raise Exception("Color and depth image do not have the same resolution.")

    # if depth.mode != "I":
    #     raise Exception("Depth image is not in intensity format")

    points = []    

    Z1 = 3 # an initial value for the Zbuffer
    for v in range(30,cv_rgb.shape[1]):
        for u in range(cv_rgb.shape[0]):
            color = cv_rgb[u,v]
            Z = cv_depth[u,v]/ scalingFactor
            # to prevent hole failure
            if Z<=0.5: 
                Z = Z1
                X = (u - cx) * Z1 / focal_length
                Y = -(v - cy) * Z1 / focal_length
            else:
                X = (u - cx) * Z / focal_length
                Y = -(v - cy) * Z / focal_length
                Z1 = Z

            points.append([X,Y,Z,color[0],color[1],color[2]])

    header = Header()
    header.frame_id = "try_pointcloud"
    fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('r', 12, PointField.FLOAT32, 1),
                PointField('g', 16, PointField.FLOAT32, 1),
                PointField('b', 20, PointField.FLOAT32, 1)]
    pointcloud = point_cloud2.create_cloud(header=header,fields=fields, points=points)
    return pointcloud


def generate_pointcloud_all_in_one(all_in_one_message, only_class=True):

    # generate pointclouds from rgb(or mono) and depth
    rgb_message=all_in_one_message.ColorImage
    depth_message=all_in_one_message.DepthImage

    cv_rgb = bridge.imgmsg_to_cv2(rgb_message, "rgb8")
    cv_depth = bridge.imgmsg_to_cv2(depth_message, "16UC1")
    

    if only_class:
        class_name = all_in_one_message.class_name_of_the_box


        number_of_object = len(class_name)
        ldx = all_in_one_message.x_position_of_the_box_DownLeft_corner    #left down x     with int(ldx[i]) to be the ldx of the i+1th object
        ldy = all_in_one_message.y_position_of_the_box_DownLeft_corner     #left down y
        rux = all_in_one_message.x_position_of_the_box_UpRight_corner      #right up x
        ruy = all_in_one_message.y_position_of_the_box_UpRight_corner     #right up y
        # add some filters

        print("element nums of the class === ", len(class_name))

        cv_depth = cv2.medianBlur(cv_depth,7)
        # if cv_rgb.size != cv_depth.size:
        #     raise Exception("Color and depth image do not have the same resolution.")

        # if depth.mode != "I":
        #     raise Exception("Depth image is not in intensity format")

        points = []
        all_x_in_one_object = ""
        all_y_in_one_object = ""
        all_z_in_one_object = ""
        total_x = []
        total_y = []
        total_z = []
        Z1 = 3 # an initial value for the Zbuffer
        for i in range(number_of_object):
            for v in range(int(ldy[i]),int(ruy[i])):
                for u in range(int(ldx[i]),int(rux[i])):

                    color = cv_rgb[v,u]
                    Z = cv_depth[v,u]/ scalingFactor
                    # to prevent hole failure
                    if Z<=0.5: 
                        Z = Z1
                        X = (u - cx) * Z1 / focal_length
                        Y = (v - cy) * Z1 / focal_length
                    else:
                        X = (u - cx) * Z / focal_length
                        Y = (v - cy) * Z / focal_length
                        Z1 = Z
                    
                    all_x_in_one_object = all_x_in_one_object + str(X)+" "
                    all_y_in_one_object = all_y_in_one_object + str(Y)+" "
                    all_z_in_one_object = all_z_in_one_object + str(Z)+" "

                    points.append([X,Y,Z,color[0],color[1],color[2]])
            total_x.append(all_x_in_one_object)
            total_y.append(all_y_in_one_object)
            total_z.append(all_z_in_one_object)
        geometry_data = output_geometry(class_name, total_x, total_y, total_z)
        header = Header()
        header.frame_id = "try_pointcloud"
        fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('r', 12, PointField.FLOAT32, 1),
                    PointField('g', 16, PointField.FLOAT32, 1),
                    PointField('b', 20, PointField.FLOAT32, 1)]
        pointcloud = point_cloud2.create_cloud(header=header,fields=fields, points=points)
        return pointcloud, geometry_data
    else:
            # add some filters

        cv_depth = cv2.medianBlur(cv_depth,7)
        # cv_depth = cv2.blur(cv_depth, (9,9))
        # cv_depth = cv2.GaussianBlur(cv_depth, (3,3), 1)
        

        # cv2.imshow("RGB", cv_rgb)
        # cv2.imshow("depth", cv_depth)
        # cv2.waitKey(1)

        # if rgb.size != depth.size:
        #     raise Exception("Color and depth image do not have the same resolution.")

        # if depth.mode != "I":
        #     raise Exception("Depth image is not in intensity format")

        points = []    
        # for v in range(rgb.size[1]):
        #     for u in range(rgb.size[0]):
        #         color = rgb.getpixel((u,v))
        #         Z = depth.getpixel((u,v)) / scalingFactor
        #         if Z==0: continue
        #         X = (u - cx) * Z / focal_length
        #         Y = (v - cy) * Z / focal_length
        #         points.append([X,Y,Z,color[0],color[1],color[2]])
        Z1 = 3 # an initial value for the Zbuffer
        for v in range(30,cv_rgb.shape[1]):
            for u in range(cv_rgb.shape[0]):
                color = cv_rgb[u,v]
                Z = cv_depth[u,v]/ scalingFactor
                # to prevent hole failure
                if Z<=1: 
                    Z = Z1
                    X = (u - cx) * Z1 / focal_length
                    Y = -(v - cy) * Z1 / focal_length
                else:
                    X = (u - cx) * Z / focal_length
                    Y = -(v - cy) * Z / focal_length
                    Z1 = Z

                points.append([X,Y,Z,color[0],color[1],color[2]])

        header = Header()
        header.frame_id = "try_pointcloud"
        fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('r', 12, PointField.FLOAT32, 1),
                    PointField('g', 16, PointField.FLOAT32, 1),
                    PointField('b', 20, PointField.FLOAT32, 1)]
        pointcloud = point_cloud2.create_cloud(header=header,fields=fields, points=points)
        return pointcloud
    

def output_geometry(class_name, total_x, total_y, total_z):
    geometry_data = position_3d()
    geometry_data.class_name_of_the_box = class_name
    geometry_data.x_positions_of_all_class = total_x
    geometry_data.y_positions_of_all_class = total_y
    geometry_data.z_positions_of_all_class = total_z
    return geometry_data


def rgb_callback(rgb_image):
    global rgb_message
    rgb_message = rgb_image



def depth_callback(depth_image):
    global depth_message
    depth_message = depth_image


def all_in_one_callback(all_in_one):
    global all_in_one_message
    all_in_one_message = all_in_one



if __name__ == '__main__':
    rospy.init_node('PointCloud_Generator', anonymous=True)

    rgb_topic='AeroCameraDown/infra2/image_rect_raw'
    depth_topic='AeroCameraDown/depth/image_rect_raw'
    all_in_one_topic='class_image_YOLO'
    point_topic='Points'
  
    # rospy.Subscriber(rgb_topic, rosImage, rgb_callback)  # BGR
    # rospy.Subscriber(depth_topic, rosImage, depth_callback)  # Depth
    rospy.Subscriber(all_in_one_topic, image_with_class, all_in_one_callback) # BGR, Depth, Class(labels and their location)
    point_pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=1)
    geometry_pub = rospy.Publisher("Geometry_Data_of_Detection", position_3d, queue_size=1)
    rospy.sleep(0.05)
    while True:
        # pointcloud = generate_pointcloud(rgb_message, depth_message)
        pointcloud, geometry_data = generate_pointcloud_all_in_one(all_in_one_message, True)
        point_pub.publish(pointcloud)
        geometry_pub.publish(geometry_data)