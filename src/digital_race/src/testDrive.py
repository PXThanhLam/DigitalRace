#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from LaneDetection import pipeline,gaussian_blur,combined_color_gradient,perspective_transform
from CarControll import CarControll
from std_msgs.msg import Float32




car=CarControll()
def rgb_image_callback(img_msg):
    try:
        np_arr = np.fromstring(img_msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = gaussian_blur(image, 3)
        combine = combined_color_gradient(image)
        warp_image, Minv, M = perspective_transform(combine)
        result, leftx, lefty, rightx, righty, img_left_fit, img_right_fit = pipeline(warp_image, 0, image, Minv)
        if img_left_fit is not None:
            car.left_fit = img_left_fit
        if img_right_fit is not None:
            car.right_fit = img_right_fit

        cte,speed, _, _, _, _, _, _ = car.driveCar(leftx, lefty, rightx, righty, car.left_fit,car.right_fit, 0)
        print str(cte)+'   '+str(speed)

        ang_pub.publish(cte)
        spe_pub.publish(np.float32(speed))
    except CvBridgeError, e:
        print 'error'








rospy.init_node('testDrive',anonymous=True)
rospy.loginfo("Get_video start!")
bridge=CvBridge()
ang_pub = rospy.Publisher("team1/set_angle",Float32,queue_size=100)
spe_pub=rospy.Publisher("team1/set_speed",Float32,queue_size=100)
rgb_subcriber=rospy.Subscriber("/team1/camera/rgb/compressed", CompressedImage, rgb_image_callback)
rospy.spin()
