import os
import time
import glob
import cv2
import numpy as np
import tensorflow as tf
from keras import backend as K
from keras.models import load_model,Model
from YoloDetectUtils import *

from skimage import feature as ft
from sklearn.externals import joblib
from HogSVMSignDetection import detect_sign
sess = K.get_session()
vehicle_index= [2,5,6,7]
vehicle_dict={2:'car',5:'bus',6:'train',7:'truck',9:'traffic light'}
sign_index=[11,12]
sign_dict={11:'sign'}

class_names= read_classes('/home/tl/catkin_ws/src/digital_race/src/yolo_coco_classes.txt')
anchors=read_anchors('/home/tl/catkin_ws/src/digital_race/src/yolo_anchors.txt')
yolo_model=load_model('/home/tl/catkin_ws/src/digital_race/src/tiny_yolo.h5')
yolo_outputs = yolo_head(yolo_model.output, anchors, len(class_names))
image_shape = np.float32(240), np.float32(320)
scores, boxes, classes = yolo_eval(yolo_outputs, image_shape=image_shape, score_threshold=.01)



def yolo_sign_car_detection(image,car_thread=.25):
    image_data = preprocess_image(image, model_image_size=(416, 416))
    out_scores, out_boxes, out_classes = sess.run([scores, boxes, classes],
                                                  feed_dict={yolo_model.input: image_data})
    vehicle_classes=[]
    vehicle_scores=[]
    vehicle_out_boxes=[]
    sign_classes=[]
    sign_scores=[]
    sign_out_boxes=[]
    for i, c in reversed(list(enumerate(out_classes))):
        if c in vehicle_index and out_scores[i]>=car_thread:
            vehicle_classes.append(c)
            vehicle_scores.append(out_scores[i])
            vehicle_out_boxes.append(out_boxes[i])
        elif c in sign_index:
            sign_classes.append(c)
            sign_scores.append(out_scores[i])
            sign_out_boxes.append(out_boxes[i])

    vehicle_sort_scores_index=np.argsort(vehicle_scores)[::-1]
    vehicle_scores=list(np.array(vehicle_scores)[vehicle_sort_scores_index])
    vehicle_classes=list(np.array(vehicle_classes)[vehicle_sort_scores_index])
    vehicle_out_boxes=list(np.array(vehicle_out_boxes)[vehicle_sort_scores_index])

    sign_sort_scores_index = np.argsort(sign_scores)[::-1]
    sign_scores = list(np.array(sign_scores)[sign_sort_scores_index])
    sign_classes = list(np.array(sign_classes)[sign_sort_scores_index])
    sign_out_boxes = list(np.array(sign_out_boxes)[sign_sort_scores_index])
    for index,out_box in enumerate(vehicle_out_boxes):
        vehicle_out_boxes[index][2]=vehicle_out_boxes[index][2]-vehicle_out_boxes[index][0]
        vehicle_out_boxes[index][3]=vehicle_out_boxes[index][3]-vehicle_out_boxes[index][1]
        top, left, height, width = out_box
        vehicle_out_boxes[index] = [left, top, width, height]
    for index,out_box in enumerate(sign_out_boxes):
        padding_small = 15
        sign_out_boxes[index][2]=sign_out_boxes[index][2]-sign_out_boxes[index][0]
        sign_out_boxes[index][3]=sign_out_boxes[index][3]-sign_out_boxes[index][1]
        top, left, height, width = out_box
        x,y,w,h=left, top, width, height
        if sign_classes[index] is 11:
            x,y,w,h=max(0,int(x-padding_small)),max(0,int(y-padding_small)),min(int(w+2*padding_small),int(320-x+padding_small)),min(int(h+2*padding_small),int(240-h+padding_small))
        elif w*h>=225:
            if w*h>=900:
                padding_small=5
            x,y,w,h=max(0,int(x-padding_small)),max(0,int(y-padding_small)),min(int(w+2*padding_small),int(320-x+padding_small)),min(int(h+2*padding_small),int(240-h+padding_small))



        sign_out_boxes[index] = [x, y, w, h,padding_small]





    return [vehicle_scores, vehicle_out_boxes, vehicle_classes],[sign_scores, sign_out_boxes, sign_classes]






























