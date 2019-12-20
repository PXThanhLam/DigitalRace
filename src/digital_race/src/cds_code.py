import cv2
import numpy as np
import matplotlib.pyplot as plt
src=np.array([[6, 5, 4],[5, 6, 1],[6, 5, 3]],dtype=np.uint8)

def get_histogram(src):
    histogram=[0]*256
    row_length=len(src)
    col_length=len(src[1])
    for row in range(row_length):
        for col in range(col_length):
            histogram[src[row][col]]+=1
    return histogram

def equalize_hist(src):
    histogram=get_histogram(src)
    sum_hist=[0]*256
    for i in range(len(histogram)):
        sum_hist[i]=sum(histogram[:i+1])
    for hist in sum_hist:
        if hist>0:
            min_sum_h=hist
            break
    row_length = len(src)
    col_length = len(src[1])
    for row in range(row_length):
        for col in range(col_length):
            src[row][col]=int((float(sum_hist[src[row][col]]-min_sum_h)/float(row_length*col_length-min_sum_h))*255)
    # for row in range(row_length):
    #     for col in range(col_length):
    #         src[row][col]=src[row][col]/src[row_length-1][col_length-1]
    return src

print equalize_hist(src)
print cv2.equalizeHist(src)

