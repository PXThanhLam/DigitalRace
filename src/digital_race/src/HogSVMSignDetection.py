import os
import numpy as np
import cv2
from skimage import feature as ft
from sklearn.externals import joblib
clf = joblib.load("./svm_model.pkl")
cls_names = ["straight", "left", "right", "stop", "nohonk", "crosswalk", "background"]
img_label = {0:"straight", 1:"left", 2:"right", 3:"stop", 4:"nohonk", 5:"crosswalk", 6:"background"}


def preprocess_img(imgBGR, erode_dilate=True):
    rows, cols, _ = imgBGR.shape
    imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
    # cv2.imshow('hsv',imgHSV)
    # cv2.imshow('bgr',imgBGR)
    # cv2.waitKey(0)

    Bmin = np.array([100, 23, 26])
    Bmax = np.array([124, 255, 255])
    img_Bbin = cv2.inRange(imgHSV, Bmin, Bmax)

    Rmin1 = np.array([0, 23, 26])
    Rmax1 = np.array([10, 255, 255])
    img_Rbin1 = cv2.inRange(imgHSV, Rmin1, Rmax1)

    Rmin2 = np.array([156, 43, 46])
    Rmax2 = np.array([180, 255, 255])
    img_Rbin2 = cv2.inRange(imgHSV, Rmin2, Rmax2)
    img_Rbin = np.maximum(img_Rbin1, img_Rbin2)
    img_bin = np.maximum(img_Bbin, img_Rbin)
    if erode_dilate is True:
        kernelErosion = np.ones((5, 5), np.uint8)
        kernelDilation = np.ones((5, 5), np.uint8)
        img_bin = cv2.erode(img_bin, kernelErosion, iterations=2)
        img_bin = cv2.dilate(img_bin, kernelDilation, iterations=2)

    return img_bin


def contour_detect(img_bin, min_area=0, max_area=-1, wh_ratio=2.0):
    rects = []
    contours,_ = cv2.findContours(img_bin.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        return rects

    max_area = img_bin.shape[0] * img_bin.shape[1] if max_area < 0 else max_area
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_area and area <= max_area:
            x, y, w, h = cv2.boundingRect(contour)
            if 1.0 * w / h < wh_ratio and 1.0 * h / w < wh_ratio:
                rects.append([x, y, w, h])
    return rects


def draw_rects_on_img(img, rects):
    img_copy = img.copy()
    cv2.rectangle(img_copy, (rects[0], rects[1]), (rects[2], rects[3]), (0, 255, 0), 2)
    return img_copy


def hog_extra_and_svm_class(proposal, clf, resize=(64, 64)):
    img = cv2.cvtColor(proposal, cv2.COLOR_BGR2GRAY)
    img = cv2.resize(img, resize)
    bins = 9
    cell_size = (8, 8)
    cpb = (2, 2)
    norm = "L1"
    features = ft.hog(img, orientations=bins, pixels_per_cell=cell_size,
                      cells_per_block=cpb, block_norm=norm, transform_sqrt=True)
    features = np.reshape(features, (1, -1))
    cls_prop = clf.predict_proba(features)
    return cls_prop

def merge_rectangle(r1,r2):
    return [min(r1[0],r2[0]),min(r1[1],r2[1]),max(r1[2],r2[2]),max(r1[3],r2[3])]



def detect_sign(yolo_single_output_frame,padding_small):
    rows, cols, _ = yolo_single_output_frame.shape
    img_bin = preprocess_img(yolo_single_output_frame,  False)
    rects = contour_detect(img_bin)
    real_rects = []
    rect_total_area=0
    for rect in rects:
        rect_total_area+=rect[2]*rect[3]
    is_noise=True if rect_total_area<50 or rows*cols>(32+2*padding_small)**2 else False
    if is_noise:
        return 'noise'#,img_bin,[]
    if rows <=2*padding_small or cols<=2*padding_small:
        return 'not_enough_image'#,img_bin,[]
    for rect in rects:
        xc = int(rect[0] + rect[2] / 2)
        yc = int(rect[1] + rect[3] / 2)

        size = max(rect[2], rect[3])
        x1 = max(0, int(xc - size / 2))
        y1 = max(0, int(yc - size / 2))
        x2 = min(cols, int(xc + size / 2))
        y2 = min(rows, int(yc + size / 2))
        if (x2 - x1) * (y2 - y1) >= 120:
            real_rects.append([x1,y1,x2,y2])
        elif (x2 - x1) * (y2 - y1) >= 75:
            if len(rects)==1:
                real_rects.append([x1-3, y1-3, x2+3, y2+3])
            else:
                real_rects.append([x1,y1,x2,y2])
    if len(real_rects) == 1:
        real_rect = real_rects[0]
    elif len(real_rects) == 2:
        real_rect = merge_rectangle(real_rects[0], real_rects[1])
    else:
        real_rect = [0, 0, yolo_single_output_frame.shape[1], yolo_single_output_frame.shape[0]]
    x1,y1,x2,y2=real_rect
    proposal = yolo_single_output_frame[y1:y2, x1:x2]
    try:
        cls_prop = hog_extra_and_svm_class(proposal, clf)[0]
    except:
        return 'noise'
    if cls_prop[1]>1.2*cls_prop[2]:
        print padding_small
        print (rows,cols)
        print 'prob: ' + str(cls_prop)
        return 'sign_detect_left'#,img_bin,real_rect
    elif cls_prop[2]>1.2*cls_prop[1]:
        print 'prob: ' + str(cls_prop)
        return 'sign_detect_right'#,img_bin,real_rect
    else:
        return 'noise'#,img_bin, real_rect





if __name__ == "__main__":
    img = cv2.imread('/home/tl/Pictures/62.png')
    print img.shape
    result,binimg,rect=detect_sign(img,15)
    cv2.imshow('image',img)
    cv2.imshow('binimg',binimg)
    print result
    if result not in ['noise','not_enough_image']:
        img_rects = draw_rects_on_img(img, rect)
        cv2.imshow("image with rects", img_rects)
    cv2.waitKey(0)



