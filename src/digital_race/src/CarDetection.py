import numpy as np
import cv2
import cv2
from LaneDetection import perspective_transform,pipeline,combined_color_gradient,detect_cross,detect_snow,hls_select
import matplotlib.pyplot as plt
import time

car_cascade = cv2.CascadeClassifier('/home/tl/catkin_ws/src/digital_race/src/car.xml')
def findIntersection(x1, y1, x2, y2, x3, y3, x4, y4):
    if ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)) ==0:
        return False
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
    return px, py


def area(x1, y1, x2, y2, x3, y3):
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)
def isInside(x1, y1, x2, y2, x3, y3, x, y):
    A = area(x1, y1, x2, y2, x3, y3)

    A1 = area(x, y, x2, y2, x3, y3)
    A2 = area(x1, y1, x, y, x3, y3)
    A3 = area(x1, y1, x2, y2, x, y)
    if abs(A -(A1 + A2 + A3))<10:
        return True
    else:
        return False

def distance_from_point_to_line(p1,p2,p3):
    return abs(np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1))
def detect_car_pos(bounding_cars,lx1,ly1,rx1,ry1,intersec_x,intersec_y):
    x,y,w,h=bounding_cars[0]
    distance_to_left_lane=distance_from_point_to_line(np.array([lx1,ly1]),np.array([intersec_x,intersec_y]),np.array([x+w/2,y+h/2]))
    distance_to_right_lane = distance_from_point_to_line(np.array([rx1, ry1]), np.array([intersec_x, intersec_y]),np.array([x + w / 2, y + h / 2]))
    return distance_to_left_lane<distance_to_right_lane

def get_car_boundary(car_cascade,image,leftx, lefty, rightx, righty,lr):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cars = car_cascade.detectMultiScale(gray, scaleFactor=1.02, minNeighbors=1,minSize=(40,40))
    #lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[int(len(leftx)/2)], lefty[int(len(leftx)/2)],rightx[0], righty[0], rightx[int(len(rightx)/2)], righty[int(len(rightx)/2)]
    #lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[-1], lefty[-1],rightx[0], righty[0], rightx[-1], righty[-1]
    lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[int(2*len(leftx)/3)], lefty[int(2*len(lefty)/3)], rightx[0], righty[0], rightx[int(2*len(rightx)/3)],righty[int(2*len(righty)/3)]
    intersec_x, intersec_y = findIntersection(lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2)
    bounding_cars=[]
    confirm_left_right='no'
    for (x, y, w, h) in cars:
        is_lr = False if lr == 'equal' else True
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(image, (x + w / 2, y + h / 2), 3, (255, 255, 0))
        if isInside(lx1,ly1,rx1,ry1,intersec_x,intersec_y,x+w/2,y+h/2):
            if  not discard_shadown_overwhelm_line([x,y,w,h],leftx,rightx) or is_lr:
                bounding_cars.append((x,y,w,h))
                #break

        elif isInside(lx1,ly1,rx1,ry1,intersec_x,intersec_y,x+w,y+h):
            bounding_cars.append((x, y, w, h))
            confirm_left_right='left'
            #break
        elif isInside(lx1,ly1,rx1,ry1,intersec_x,intersec_y,x,y+h):
            bounding_cars.append((x, y, w, h))
            confirm_left_right = 'right'
            #break


    return bounding_cars,confirm_left_right

#shadow_box : x,y,w,h
def discard_shadown_overwhelm_line(shadow_box,listleft_x,listright_x):
   if abs(abs(listleft_x[int(len(listleft_x)/2)]-shadow_box[0])- abs(shadow_box[0]+shadow_box[2]-listright_x[int(len(listright_x)/2)]))<30 :#and shadow_box[2]>abs(listright_x[-1]-listleft_x[-1]):
       return True
   return False


if __name__=='__main__':
    img_path='/home/tl/Pictures/Cardetec/detec13.png'
    image=cv2.imread(img_path)
    image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
    combine = combined_color_gradient(image)
    warp_image,Minv,_=perspective_transform(combine)
    leftx, lefty, rightx, righty, img_left_fit, img_right_fit,lre = pipeline(warp_image, 0, image, Minv)
    #lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[int(2 * len(leftx) / 3)], lefty[int(2 * len(leftx) / 3)], rightx[0], righty[0], rightx[int(2 * len(rightx) / 3)], righty[int(2 * len(rightx) / 3)]
    #lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[-1], lefty[-1], rightx[0], righty[0], rightx[-1], righty[-1]
    lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[int(2 * len(leftx) / 3)], lefty[int(2 * len(lefty) / 3)], rightx[0], righty[0], rightx[int(2 * len(rightx) / 3)], righty[int(2 * len(righty) / 3)]

    bouding_cars,confirm=get_car_boundary(car_cascade,image,leftx, lefty, rightx, righty,lre)
    intersec_x,intersec_y=findIntersection(lx1,ly1,lx2,ly2,rx1,ry1,rx2,ry2)
    pt1=tuple(np.asarray((lx1,ly1),dtype=np.int))
    pt2=tuple(np.asarray((rx1,ry1),dtype=np.int))
    pt3=tuple(np.asarray((intersec_x,intersec_y),dtype=np.int))
    cv2.line(image,pt1,pt3,(0, 255, 0),2)
    cv2.line(image,pt2,pt3,(0, 255, 0),2)

    for (x, y, w, h) in bouding_cars:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.circle(image,(x+w/2,y+h/2),3,(255,255,0))
        print (x,y,w,h)
        print confirm
        print '-----------'
    print lre
    cv2.imshow("image", image)
    cv2.waitKey()

    # cap = cv2.VideoCapture('output.avi')
    # while (cap.isOpened()):
    #     ret, image = cap.read()
    #     if ret == True:
    #         start_time = time.time()
    #         image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
    #         combine = combined_color_gradient(image)
    #         warp_image, Minv, _ = perspective_transform(combine)
    #         leftx, lefty, rightx, righty, img_left_fit, img_right_fit, lre = pipeline(warp_image, 0, image, Minv)
    #         if len(leftx)>0 and len(rightx)>0:
    #             lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2 = leftx[0], lefty[0], leftx[int(len(leftx) / 2)], lefty[
    #                 int(len(leftx) / 2)], rightx[0], righty[0], rightx[int(len(rightx) / 2)], righty[int(len(rightx) / 2)]
    #             bouding_cars, confirm = get_car_boundary(car_cascade, image, leftx, lefty, rightx, righty,lre)
    #             intersec_x, intersec_y = findIntersection(lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2)
    #             pt1 = tuple(np.asarray((lx1, ly1), dtype=np.int))
    #             pt2 = tuple(np.asarray((rx1, ry1), dtype=np.int))
    #             pt3 = tuple(np.asarray((intersec_x, intersec_y), dtype=np.int))
    #             cv2.line(image, pt1, pt3, (0, 255, 0), 2)
    #             cv2.line(image, pt2, pt3, (0, 255, 0), 2)
    #             for (x, y, w, h) in bouding_cars:
    #                 cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
    #                 cv2.circle(image, (x + w / 2, y + h / 2), 3, (255, 255, 0))
    #             cv2.imshow("image", image)
    #             cv2.waitKey(1)
    #             #print "--- frame ---" +str(1/(time.time() - start_time))
