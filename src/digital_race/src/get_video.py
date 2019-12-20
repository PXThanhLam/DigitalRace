#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError






def rgb_image_callback(img_msg):
    try:
        np_arr = np.fromstring(img_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # print 'rgb'
        # print np.asarray(cv_image).shape
        # cv2.imshow('rgb', cv_image)
        # cv2.waitKey(125)
        out.write(cv_image)
    except CvBridgeError, e:
        print 'error'


def depth_image_callback(img_msg):
    try:
        np_arr = np.fromstring(img_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print 'depth'
        print np.asarray(cv_image).shape
        # cv2.imshow('depth', cv_image)
        # cv2.waitKey(120)
        out.write(cv_image)
    except CvBridgeError, e:
        print 'error'




rospy.init_node('get_video',anonymous=True)
rospy.loginfo("Get_video start!")
bridge=CvBridge()

#rgb_subcriber=rospy.Subscriber("/team1/camera/rgb/compressed", CompressedImage, rgb_image_callback)
depth_subcriber=rospy.Subscriber("/team1/camera/rgb/compressed", CompressedImage, rgb_image_callback)
out = cv2.VideoWriter('outputm3.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 20, (320,240))
rospy.spin()
# cv2.destroyWindow('rgb')
# cv2.destroyWindow('depth')