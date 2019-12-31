import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

car_cascade = cv2.CascadeClassifier('/home/tl/catkin_ws/src/digital_race/src/car.xml')
def cascade_get_car_position(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cars = car_cascade.detectMultiScale(gray, scaleFactor=1.06, minNeighbors=1, minSize=(40, 40))
    return cars

