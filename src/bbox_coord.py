#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from detection_msgs.msg import BoundingBox,BoundingBoxes
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def bbox(data):
    c_Z = 239
    c_Y = 319

    for box in data.bounding_boxes:
        center_y = (box.ymin+box.ymax)//2
        center_x = (box.xmin+box.xmax)//2
        depth_array = np.array(depth_image, dtype=np.float32)/1000 # mm to m
        Y_pix = center_x - c_Y
        Z_pix = center_y - c_Z
        theta = np.arctan2(np.sqrt(Y_pix**2+Z_pix**2),640)

        X_coord = depth_array[center_y, center_x]*np.cos(theta) # projection
        Y_coord = -1*Y_pix/640*X_coord
        Z_coord = -1*Z_pix/640*X_coord

        rospy.loginfo(
            "\n Obj : {} \n depth : {:.3f}m \n X : {:.3f}m \n Y : {:.3f}m \n Z : {:.3f}m \n".format(
                box.Class, depth_array[center_y, center_x], X_coord, Y_coord, Z_coord
            )
        )

def convert_depth_image(image):
    bridge = CvBridge()
    try:
        global depth_image
        depth_image = bridge.imgmsg_to_cv2(image, image.encoding)
    except CvBridgeError as e:
        print(e)

def bbox_coord():
    rospy.init_node('bbox_coord', anonymous=True)
    rospy.Subscriber('/yolov5/detections',BoundingBoxes,bbox,queue_size=1)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,convert_depth_image,queue_size=1)
    rospy.spin()

if __name__=="__main__":
    bbox_coord()

# if np.sqrt(np.square(X_coord)+np.square(Y_coord)+np.square(Z_coord))<0.23:
#     goPath(X_coord,Y_coord,Z_coord)