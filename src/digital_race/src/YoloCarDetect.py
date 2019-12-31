import os
import time
import glob
import cv2
import numpy as np
import tensorflow as tf
from keras import backend as K
from keras.models import load_model,Model
from YoloDetectUtils import *
sess = K.get_session()
vehicle_index= [11]#[2,5,6,7]
vehicle_dict={11:'stop sign'}#{2:'car',5:'bus',6:'train',7:'truck',9:'traffic light',11:'stop sign'}
class_names= read_classes('/home/tl/catkin_ws/src/digital_race/src/yolo_coco_classes.txt')
anchors=read_anchors('/home/tl/catkin_ws/src/digital_race/src/yolo_anchors.txt')
yolo_model=load_model('/home/tl/catkin_ws/src/digital_race/src/tiny_yolo.h5')
yolo_outputs = yolo_head(yolo_model.output, anchors, len(class_names))
image_shape = np.float32(240), np.float32(320)
scores, boxes, classes = yolo_eval(yolo_outputs, image_shape=image_shape, score_threshold=.05)



def yolo_car_detection(image):
    image_data = preprocess_image(image, model_image_size=(416, 416))
    out_scores, out_boxes, out_classes = sess.run([scores, boxes, classes],
                                                  feed_dict={yolo_model.input: image_data})

    vehicle_classes=[]
    vehicle_scores=[]
    vehicle_out_boxes=[]

    for i, c in reversed(list(enumerate(out_classes))):
        if c in vehicle_index:
            vehicle_classes.append(c)
            vehicle_scores.append(out_scores[i])
            vehicle_out_boxes.append(out_boxes[i])
    if len(vehicle_scores)>0:
        print '---Box Found-----'
    for idx,vehicle in enumerate(vehicle_classes):
        print vehicle_dict[vehicle] +str(vehicle_scores[idx])
    for index,out_box in enumerate(vehicle_out_boxes):
        vehicle_out_boxes[index][2]=vehicle_out_boxes[index][2]-vehicle_out_boxes[index][0]
        vehicle_out_boxes[index][3]=vehicle_out_boxes[index][3]-vehicle_out_boxes[index][1]
        top, left, height, width = out_box
        vehicle_out_boxes[index] = [left, top, width, height]

    return vehicle_scores, vehicle_out_boxes, vehicle_classes

if __name__ == "__main__":

    # start = time.time()
    # image = cv2.imread('/home/tl/Pictures/Cardetec/detec5.png')
    # yolo_car_detection(image,colors)
    # print 'totlal run time ' + str(time.time() - start)

    cap = cv2.VideoCapture('output.avi')
    while (cap.isOpened()):
        start = time.time()
        ret, image = cap.read()

        if ret:
            vehicle_scores, vehicle_out_boxes, vehicle_classes = yolo_car_detection( image)
            for (x, y, w, h) in vehicle_out_boxes:
                x,y,w,h=int(x),int(y),int(w),int(h)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(image, (x + w / 2, y + h / 2), 3, (255, 255, 0))
            end = time.time()

            # fps
            t = end - start
            fps = "Fps: {:.2f}".format(1 / t)
            # display a piece of text to the frame
            cv2.putText(image, fps, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow('image', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break




























