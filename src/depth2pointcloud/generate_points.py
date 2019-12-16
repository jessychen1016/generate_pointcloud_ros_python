#!/usr/bin/python
import sys
import os
from PIL import Image
import numpy
import rospy
import roslib
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import Image as rosImage
import cv2


bridge = CvBridge()


focal_length = 378.9499338662045
cx = 321.5308027037405
cy = 242.8304397646636
scalingFactor = 1


def generate_pointcloud(rgb_message, depth_message):

    # generate pointclouds from rgb(or mono) and depth
    print("encoding", rgb_message.encoding)
    cv_rgb = bridge.imgmsg_to_cv2(rgb_message, "rgb8")
    cv_depth = bridge.imgmsg_to_cv2(depth_message, "16UC1")
    
    rgb = Image.fromarray(cv_rgb)
    depth = Image.fromarray(cv_depth)

    print("RGB SIZE", rgb.size)
    print("DEPTH SIZE", depth.size)
    if rgb.size != depth.size:
        raise Exception("Color and depth image do not have the same resolution.")

    # if depth.mode != "I":
    #     raise Exception("Depth image is not in intensity format")

    points = []    
    for v in range(rgb.size[1]):
        for u in range(rgb.size[0]):
            color = rgb.getpixel((u,v))
            Z = depth.getpixel((u,v)) / scalingFactor
            if Z==0: continue
            X = (u - cx) * Z / focal_length
            Y = (v - cy) * Z / focal_length
            points.append([X,Y,Z,color[0],color[1],color[2]])
    header = Header()
    header.frame_id = "try_pointcloud"
    fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('r', 12, PointField.FLOAT32, 1),
                PointField('g', 16, PointField.FLOAT32, 1),
                PointField('b', 20, PointField.FLOAT32, 1),]
    pointcloud = point_cloud2.create_cloud(header=header,fields=fields, points=points)
    return pointcloud
    

def rgb_callback(rgb_image):
    global rgb_message
    rgb_message = rgb_image
    print(type(rgb_message))
    print("UUUUUUUUUUUUUUUUUUUUUUUUU")


def depth_callback(depth_image):
    global depth_message
    depth_message = depth_image



if __name__ == '__main__':
    rospy.init_node('PointCloud_Generator', anonymous=True)

    rgb_topic='AeroCameraDown/infra2/image_rect_raw'
    depth_topic='AeroCameraDown/depth/image_rect_raw'
    point_topic='Points'
  
    rospy.Subscriber(rgb_topic, rosImage, rgb_callback)  # BGR
    rospy.Subscriber(depth_topic, rosImage, depth_callback)  # Depth
    rospy.sleep(2.0)
    point_pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=1)

    while True:
        pointcloud = generate_pointcloud(rgb_message, depth_message)
        point_pub.publish(pointcloud)
        rospy.sleep(1.0)

