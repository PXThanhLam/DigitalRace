#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from LaneDetection import pipeline,gaussian_blur,combined_color_gradient,perspective_transform,detect_cross,detect_snow,hls_select
from CarDetection import get_car_boundary,findIntersection
from CarControll import CarControll
from std_msgs.msg import Float32
import time



car=CarControll()
car_cascade = cv2.CascadeClassifier('/home/tl/catkin_ws/src/digital_race/src/car.xml')
def rgb_image_callback(img_msg):
    try:
        start_time = time.time()
        np_arr = np.fromstring(img_msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        S_binary_img = hls_select(image)
        image = gaussian_blur(image, 3)
        combine = combined_color_gradient(image)
        Is_cross = detect_cross(combine)
        Is_snow=detect_snow(S_binary_img)
        warp_image, Minv, M = perspective_transform(combine) if not Is_cross and not Is_snow else perspective_transform(combine, np.float32([[0, 200], [80, 40], [240, 40], [320, 240]]),np.float32([[0, 240], [80, 0], [240, 0], [320, 240]]))
        leftx, lefty, rightx, righty, img_left_fit, img_right_fit,lre = pipeline(warp_image, 0, image, Minv, Is_cross,Is_snow)
        boundary_cars,confirmlr=[],'no'
        if len(leftx)>10 and len(rightx) >10:
            boundary_cars,confirmlr=get_car_boundary(car_cascade,image,leftx,lefty,rightx,righty,lre)
        if Is_cross or Is_snow:
            boundary_cars,confirmlr=[],'no'
        cte, speed, left_x_point, left_y_point, right_x_point, right_y_point, mid_x, mid_y= car.driveCar(leftx, lefty, rightx, righty, img_left_fit,img_right_fit,boundary_cars,lre,confirmlr)

        if Is_snow:
            speed=np.float32(20)
        if len(boundary_cars)>0:
            x,y,w,h=boundary_cars[0]
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)


        if len(leftx) > 0 :#and car.left_fit is None:
            for x, y in zip(leftx, lefty):
                cv2.circle(image, (x, y), 3, (0, 255, 0))
        else:
            for y in range(100, 200, 1):
                x = np.float32(car.left_fit[0] * y ** 2 + car.left_fit[1] * y + car.left_fit[2])
                cv2.circle(image, (x, y), 3, (0, 255, 0))

        if len(rightx) > 0 :#and car.right_fit is None:
            for x, y in zip(rightx, righty):
                cv2.circle(image, (x, y), 3, (0, 255, 0))
        else:
            for y in range(100, 200, 1):
                x = np.float32(car.right_fit[0] * y ** 2 + car.right_fit[1] * y + car.right_fit[2])
                cv2.circle(image, (x, y), 3, (0, 255, 0))

        cv2.circle(image, (left_x_point, left_y_point), 10, (255, 0, 0))
        cv2.circle(image, (right_x_point, right_y_point), 10, (255, 0, 0))
        cv2.circle(image, (mid_x, mid_y), 10, (255, 0, 0))
        #lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[int(len(leftx) / 2)], lefty[int(len(leftx) / 2)], rightx[0], righty[0], rightx[int(len(rightx) / 2)], righty[int(len(rightx) / 2)]

        # lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[-1], lefty[-1],rightx[0], righty[0], rightx[-1], righty[-1]
        lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[int(2*len(leftx)/3)], lefty[int(2*len(lefty)/3)], rightx[0], righty[0], rightx[int(2*len(rightx)/3)],righty[int(2*len(righty)/3)]

        intersec_x, intersec_y = findIntersection(lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2)
        pt1=tuple(np.asarray((leftx[0],lefty[0]),dtype=np.int))
        pt2=tuple(np.asarray((rightx[0],righty[0]),dtype=np.int))
        pt3=tuple(np.asarray((intersec_x,intersec_y),dtype=np.int))
        cv2.line(image,pt1,pt3,(0, 255, 0),2)
        cv2.line(image,pt2,pt3,(0, 255, 0),2)
        cv2.imshow('Frame', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
        ang_pub.publish(cte)
        spe_pub.publish(np.float32(speed))
        #print("--- %s seconds ---" % (time.time() - start_time))
    except CvBridgeError, e:
        print 'error'








rospy.init_node('testDrive',anonymous=True)
rospy.loginfo("Get_video start!")
bridge=CvBridge()
ang_pub = rospy.Publisher("team1/set_angle",Float32,queue_size=100)
spe_pub=rospy.Publisher("team1/set_speed",Float32,queue_size=100)
rgb_subcriber=rospy.Subscriber("/team1/camera/rgb/compressed", CompressedImage, rgb_image_callback)
rospy.spin()
