import numpy as np
import cv2
import matplotlib.pyplot as plt
from CarControll import CarControll

def gaussian_blur(img,kernel_size):
    return cv2.GaussianBlur(img,(kernel_size,kernel_size),0)

def abs_sobel_thresh(gray,orient='x',sobel_kernel=15,min=40,max=170):
    if orient=='x':
        abs_sobel=np.absolute(cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=sobel_kernel))
    if orient=='y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1,ksize=sobel_kernel))

    scaled_sobel=np.uint8(255*abs_sobel/np.max(abs_sobel))
    grad_binary = np.zeros_like(scaled_sobel)
    grad_binary[(scaled_sobel >= min) & (scaled_sobel <= max)]=1
    return grad_binary


def mag_thresh(gray, sobel_kernel=9, mag_thresh=(70, 190)):
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0,ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1,ksize=sobel_kernel)
    gradmag = np.sqrt(sobelx ** 2 + sobely ** 2)
    scale_factor = np.max(gradmag) / 255
    gradmag = (gradmag / scale_factor).astype(np.uint8)
    mag_binary = np.zeros_like(gradmag)
    mag_binary[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

    return mag_binary

def dir_threshold(gray, sobel_kernel=15, thresh=(0.2, 1.4)):
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    dir_binary =  np.zeros_like(absgraddir)
    dir_binary[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1
    return dir_binary

def hls_select(img, thresh=(40, 190)):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    H = hls[:,:,0]
    L = hls[:,:,1]
    S = hls[:,:,2]
    binary_output = np.zeros_like(S)
    binary_output[(S > thresh[0]) & (S <= thresh[1])] = 1
    return binary_output

def morph_transform(img,l1,r1,l2,r2,top):
    if top:
        kernel = np.ones((5,5), np.uint8)
        morph_image_left = img[0:l1, 0:r1]
        morph_image_left = cv2.morphologyEx(morph_image_left, cv2.MORPH_OPEN, kernel)
        img[0:l1, 0:r1] = morph_image_left

        morph_image_right = img[0:l2, r2:320]
        morph_image_right = cv2.morphologyEx(morph_image_right, cv2.MORPH_OPEN, kernel)
        img[0:l2, r2:320] = morph_image_right
    else:
        kernel = np.ones((5, 5), np.uint8)
        morph_image_left = img[l1:240, 0:r1]
        morph_image_left = cv2.morphologyEx(morph_image_left, cv2.MORPH_OPEN, kernel)
        img[l1:240, 0:r1] = morph_image_left

        morph_image_right = img[l2:240, r2:320]
        morph_image_right = cv2.morphologyEx(morph_image_right, cv2.MORPH_OPEN, kernel)
        img[l2:240, r2:320] = morph_image_right


    return img
def remove_middle(img,middleleft,middleright):
    img[:,middleleft:middleright]=0
    return img
def combined_color_gradient(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    #gray=cv2.equalizeHist(gray)
    gradx = abs_sobel_thresh(gray, orient='x')
    grady = abs_sobel_thresh(gray, orient='y')
    mag_binary = mag_thresh(gray)
    dir_binary = dir_threshold(gray)
    gradient = np.zeros_like(dir_binary)
    color = hls_select(image)
    use_color = (color == 1)
    use_dinary=(dir_binary == 1)
    gradient[(gradx == 1) & (grady == 1)|((mag_binary == 1) & use_dinary)] = 1
    combined = np.zeros_like(gradient)
    combined[(gradient == 1)] = 1
    combined=morph_transform(combined,100,100,100,220,True)
    return combined

def perspective_transform(image,src=np.float32([[0,200],[80,100],[240,100],[320,200]]),dst=np.float32([[50,240],[80,0],[240,0],[300,240]])):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    img_size=(image.shape[1],image.shape[0])
    warped = cv2.warpPerspective(image, M, img_size,flags=cv2.INTER_LINEAR)
    warped=morph_transform(warped,80,80,80,240,True)
    return warped,Minv,M

def detect_snow(sbinary_image):
    histogram=np.sum(sbinary_image[100:240],axis=1)
    if len(np.where(histogram>=160)[0])>40:
        return True
    return False

def detect_cross(bin_img):
    x_histogram=np.sum(bin_img,axis=1)
    if 100-len(np.where(x_histogram[100:200]>0)[0]) >25:
        return True
    else:
        return False


def get_left_right_base(bin_img,top,bot):
    histogram=np.sum(bin_img[top:bot,:],axis=0)
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    if histogram[rightx_base]<=8:
        print 'right_hist ' +str(histogram[rightx_base])
        rightx_base=290
    if histogram[leftx_base]<=8:
        print 'left hist ' +str(histogram[leftx_base])
        leftx_base=30
    if rightx_base-leftx_base<70:
        #print 'Before :'+' '+str([leftx_base,rightx_base])
        left_bias=True if midpoint-leftx_base > rightx_base-midpoint+15 else False
        right_bias=True if midpoint-leftx_base+15 < rightx_base-midpoint else False

        if not left_bias and not right_bias:
            check_hist=np.sum(bin_img,axis=0) #if top <10 else histogram
            left_bias=True if np.sum(check_hist[:midpoint]) >= np.sum(check_hist[midpoint:]) else False
        if left_bias:
            rightx_base=np.argmax(histogram[rightx_base+80:]) + rightx_base+80
            #print 'After :' + ' ' + str([leftx_base, rightx_base])
        else :
            leftx_base=np.argmax(histogram[:leftx_base-80])
            #print 'After :' + ' ' + str([leftx_base, rightx_base])
    return leftx_base,rightx_base

def pipeline(binary_warped, count, image,Minv,Is_cross=False,Is_snow=False):

    if count == 0:

        top_x=160
        if  Is_snow :
            binary_warped[100:240]=0


        leftx_base,rightx_base=get_left_right_base(binary_warped,top_x,240) if not Is_cross and not Is_snow else get_left_right_base(binary_warped,0,100)
        y_bot=binary_warped.shape[0] if not Is_cross and not Is_snow else 100


        nwindows =50  if not Is_cross and not Is_snow else 30
        window_height = np.int(y_bot / nwindows)

        nonzero = binary_warped.nonzero()

        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin=30 if not Is_cross and not Is_snow else 20
        minpix =3
        left_lane_inds = []
        right_lane_inds = []
        for window in range(nwindows):

            win_y_low = y_bot - (window + 1) * window_height
            win_y_high = y_bot - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            cv2.rectangle(binary_warped, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),(0, 255, 0), 2)
            cv2.rectangle(binary_warped, (win_xright_low, win_y_low), (win_xright_high, win_y_high),(0, 255, 0), 2)


            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]

            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            cv2.imshow('c', binary_warped)
            cv2.waitKey(0)

            left_lane_inds.append(good_left_inds[-5:]) if len(good_left_inds)>minpix*2  else left_lane_inds.append([])
            right_lane_inds.append(good_right_inds[:5]) if len(good_right_inds)>minpix*2 else right_lane_inds.append([])

            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds[-10:]]))


            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds[:10]]))

        left_right_equal='equal'
        if rightx_base-leftx_base<170 and not Is_snow and not Is_cross:
            left_zero_count=0
            for inds in left_lane_inds:
                if len(inds)==0:
                    left_zero_count+=1
            right_zero_count = 0
            for inds in right_lane_inds:
                if len(inds) == 0:
                    right_zero_count += 1
            left_right_equal = 'left' if (right_zero_count-left_zero_count) > nwindows /5 else 'right' if (left_zero_count-right_zero_count) > nwindows /5 else 'equal'

        # print left_right_equal
        print rightx_base-leftx_base

        left_lane_inds = np.concatenate(left_lane_inds).astype(np.int32)
        right_lane_inds = np.concatenate(right_lane_inds).astype(np.int32)
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]



        pts_left = np.array([np.transpose(np.vstack([leftx, lefty]))],dtype='float32')
        pts_right = np.array([np.transpose(np.vstack([rightx,righty]))],dtype='float32')

        pts_img_left=cv2.perspectiveTransform(pts_left, Minv) if len(pts_left[0])>20 else None
        pts_img_right=cv2.perspectiveTransform(pts_right,Minv) if len(pts_right[0])>20 else None


        left_fit = np.polyfit(pts_img_left[0][:, 1], pts_img_left[0][:, 0], 2) if len(pts_left[0])>40 else None
        right_fit = np.polyfit(pts_img_right[0][:, 1], pts_img_right[0][:, 0], 2) if len(pts_right[0])>40 else None


        left_x,left_y = ([],[]) if pts_img_left is None else (pts_img_left[0][:,0],pts_img_left[0][:,1])
        right_x,right_y = ([],[]) if pts_img_right is None else (pts_img_right[0][:,0],pts_img_right[0][:,1])

        return left_x,left_y,right_x,right_y,left_fit,right_fit,left_right_equal



if __name__=='__main__':
    # cap = cv2.VideoCapture('output.avi')
    # car = CarControll()
    # while (cap.isOpened()):
    #     ret, image = cap.read()
    #     if ret == True:
    #         image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
    #         S_binary_img = hls_select(image)
    #         image=gaussian_blur(image,3)
    #         combine = combined_color_gradient(image)
    #         Is_cross= detect_cross(combine)
    #         Is_snow=detect_snow(S_binary_img)
    #         if Is_snow:
    #             print 'snow'
    #         warp_image,Minv,M=perspective_transform(combine) if not Is_cross and not Is_snow else perspective_transform(combine,np.float32([[0,200],[80,40],[240,40],[320,240]]),np.float32([[0,240],[80,0],[240,0],[320,240]]))
    #         leftx, lefty, rightx, righty, img_left_fit, img_right_fit = pipeline(warp_image, 0, image, Minv,Is_cross,Is_snow)
    #         left_fit=None
    #         right_fit=None
    #         if img_left_fit is not None:
    #             left_fit = img_left_fit
    #         if img_right_fit is not None:
    #             right_fit = img_right_fit
    #
    #         cte,_,left_x_point, left_y_point, right_x_point, right_y_point,mid_x,mid_y=car.driveCar(leftx, lefty, rightx, righty,left_fit,right_fit)
    #
    #         #print str(len(leftx))+'   '+str(len(rightx))
    #
    #         if len(leftx)>0 :
    #             for x,y in zip(leftx,lefty):
    #                 cv2.circle(image, (x, y), 3, (0, 255, 0))
    #         else:
    #             for y in range(100,200,1):
    #                 x=np.float32(car.left_fit[0]*y**2+car.left_fit[1]*y+car.left_fit[2])
    #                 cv2.circle(image, (x, y), 3, (0, 255, 0))
    #
    #         if len(rightx)>0 :
    #             for x,y in zip(rightx,righty):
    #                 cv2.circle(image, (x, y), 3, (0, 255, 0))
    #         else:
    #             for y in range(100,200,1):
    #                 x=np.float32(car.right_fit[0]*y**2+car.right_fit[1]*y+car.right_fit[2])
    #                 cv2.circle(image, (x, y), 3, (0, 255, 0))
    #
    #
    #
    #
    #
    #         cv2.circle(image, (left_x_point, left_y_point), 10, (255, 0, 0))
    #         cv2.circle(image, (right_x_point, right_y_point), 10, (255, 0, 0))
    #         cv2.circle(image,(mid_x,mid_y),10,(255,0,0))
    #         cv2.putText(image,str(cte),(20,20), cv2.FONT_HERSHEY_SIMPLEX,1,1)
    #
    #
    #         cv2.imshow('Frame', image)
    #         #cv2.imshow('imf',np.float32(S_binary_img))
    #         if cv2.waitKey(10) & 0xFF == ord('q'):
    #             break
    #     else:
    #         break
    #
    #
    # cap.release()
    # cv2.destroyAllWindows()


    img_path='/home/tl/Pictures/Cardetec/detec12.png'
    image = cv2.imread(img_path)
    image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
    S_binary_img=hls_select(image)
    combine = combined_color_gradient(image)

    Is_cross= detect_cross(combine)
    Is_snow=detect_snow(S_binary_img)

    warp_image,Minv,M=perspective_transform(combine) if not Is_cross and not Is_snow else perspective_transform(combine,np.float32([[0,200],[80,40],[240,40],[320,240]]),np.float32([[0,240],[80,0],[240,0],[320,240]]))
    leftx, lefty, rightx, righty, img_left_fit, img_right_fit,_ = pipeline(warp_image, 0, image, Minv,Is_cross,Is_snow)
    if len(leftx) > 0:
        for x,y in zip(leftx,lefty):
            cv2.circle(image, (x, y), 1, (0, 255, 0))

    if len(rightx)>0:
        for x,y in zip(rightx,righty):
            cv2.circle(image, (x, y), 1, (0, 255, 0))

    cv2.imshow('im',image)
    cv2.waitKey(0)

    view=False
    if view:
        gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        gray=cv2.equalizeHist(gray)
        sobelx = abs_sobel_thresh(gray, 'x')
        sobely = abs_sobel_thresh(gray, 'y')
        mag_sobel = mag_thresh(gray)
        dir_sobel = dir_threshold(image)
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        H = hls[:, :, 0]
        L = hls[:, :, 1]
        S = hls[:, :, 2]
        s_binary = hls_select(image)

        fig, axes = plt.subplots(3, 4, figsize=(20, 20))
        axes = axes.ravel()
        axes[0].set_title('Original')
        axes[0].imshow(image)
        axes[1].set_title('Sobel X gradient')
        axes[1].imshow(sobelx, cmap='gray')
        axes[2].set_title('Sobel Y gradient')
        axes[2].imshow(sobely, cmap='gray')
        axes[3].set_title('Magnitude Thresholding')
        axes[3].imshow(mag_sobel, cmap='gray')
        axes[4].set_title('Direction Thresholding')
        axes[4].imshow(dir_sobel, cmap='gray')
        axes[5].set_title('HLS Image', fontsize=10)
        axes[5].imshow(hls)
        axes[6].set_title('H Image', fontsize=10)
        axes[6].imshow(H)
        axes[7].set_title('L Image', fontsize=10)
        axes[7].imshow(L)
        axes[8].set_title('S Image', fontsize=10)
        axes[8].imshow(S)
        axes[9].set_title('S BINARY', fontsize=10)
        axes[9].imshow(s_binary)
        axes[10].set_title('COMBINE')
        axes[10].imshow(combine,cmap='gray')
        axes[11].set_title('Perpective')
        axes[11].imshow(warp_image)
        plt.show()













