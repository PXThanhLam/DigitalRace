import cv2
#from LaneDetection import pipeline
import numpy as np
import math


def bb_intersection_over_union(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou
pi = 3.141592654
patience_spped_thread=5
patience_cross_thread=0
patience_car_thread=0
class CarControll:
    def __init__(self):
        self.x_car=160
        self.y_car=240
        self.left_fit=None
        self.right_fit=None

        self.t_kP=0.95
        self.t_kI=0.0001
        self.t_kD=0.02
        self.pre_cte=0

        self.P=0
        self.I=0
        self.D=0

        self.is_slowDown=False
        self.current_velocity=40
        self.patience_speed_frame=0
        self.patience_cross_frame=0
        self.begin=True
        self.midpoint=[]
        self.cars=None
        self.patience_cars_frame=0
        self.cross_history=[]


    def calcultate_cte(self,dst_x,dst_y):
        if (dst_x==self.x_car):
            return 0
        elif (dst_y==self.y_car):
            return -60 if dst_x<self.x_car else 60

        else:
            dx=dst_x - self.x_car
            dy=self.y_car - dst_y
            if dx<0:
                angleControll=-math.atan(float(-dx)/dy)*180/pi
            else:
                angleControll=math.atan(float(dx)/dy) *180/pi

            return angleControll

    def Get_Speed_By_Cte(self,cte):
        speed=0
        if -5<= cte and cte <=5 :
            speed= 70
        elif -10<=cte and cte <=10:
            speed= 60
        elif -20 <=cte and cte <=20:
            speed= 35
        elif -25 <=cte and cte<=25:
            speed= 30
        elif -35 <=cte and cte <=35:
            speed= 25
        else:
            speed=20
        return speed

    def get_Current_speed(self,speed):
        if self.current_velocity < speed:
            if self.patience_speed_frame<=patience_spped_thread:
                self.patience_speed_frame+=1
            else :
                self.patience_speed_frame=0
                self.current_velocity=speed
        else:
            self.patience_speed_frame=0
            self.current_velocity=speed
        return self.current_velocity




    def PID(self,cte):
        self.P=cte
        self.I+=cte
        self.D=cte-self.pre_cte
        pid_cte=self.t_kP*self.P +self.t_kD*self.D +self.I*self.t_kI
        if pid_cte>60:
            pid_cte=60
        if pid_cte<-60:
            pid_cte=-60

        return pid_cte


    def driveCar(self,left_x,left_y,right_x,right_y,left_fit,right_fit,bounding_cars,lre,confirmlr,Is_cross,sign_directions):
        list_left_x=left_x
        list_left_y=left_y
        list_right_x=right_x
        list_right_y=right_y
        if (left_fit is not None) or self.begin:
            self.left_fit = left_fit
        if (right_fit is not None) or self.begin:
            self.right_fit = right_fit
        left_num=len(left_x)//2 if len(left_x) >0 else 0
        right_num=len(right_x)//2 if len(right_x)>0 else 0


        if(left_num>10 and right_num>10):
            if self.patience_cross_frame<patience_cross_thread:
                self.patience_cross_frame+=1
            if self.patience_cross_frame==patience_cross_thread:
                self.begin=False
            left_x = left_x[left_num - 1]
            left_y = left_y[left_num - 1]
            right_x = right_x[right_num - 1]
            right_y = right_y[right_num - 1]
            if self.begin or self.patience_cross_frame==patience_cross_thread:
                cte=self.calcultate_cte(float(left_x+right_x)/2,float(left_y+right_y)/2)
            else:
                midpoint=np.asarray(self.midpoint)
                cte=self.calcultate_cte(float(np.mean(midpoint[:,0])),float(np.mean(midpoint[:,1])))


        elif left_num >20 :
            if self.patience_cross_frame < patience_cross_thread:
                self.patience_cross_frame += 1
            left_x = left_x[left_num - 1]
            left_y = left_y[left_num - 1]
            right_y = np.float32(150)
            right_x = np.float32(self.right_fit[0] * right_y ** 2 + self.right_fit[1] * right_y + self.right_fit[2])
            if self.patience_cross_frame == patience_cross_thread:
                cte = self.calcultate_cte(float(left_x + right_x) / 2, float(left_y + right_y) / 2)
            else:
                midpoint = np.asarray(self.midpoint)
                cte = self.calcultate_cte(float(np.mean(midpoint[:, 0])), float(np.mean(midpoint[:, 1])))

        elif right_num>20 :
            if self.patience_cross_frame < patience_cross_thread:
                self.patience_cross_frame += 1
            right_x = right_x[right_num - 1]
            right_y = right_y[right_num - 1]
            left_y = np.float32(150)
            left_x = np.float32(self.left_fit[0] * left_y ** 2 + self.left_fit[1] * left_y + self.left_fit[2])
            if self.patience_cross_frame == patience_cross_thread:
                cte = self.calcultate_cte(float(left_x + right_x) / 2, float(left_y + right_y) / 2)
            else:
                midpoint = np.asarray(self.midpoint)
                cte = self.calcultate_cte(float(np.mean(midpoint[:, 0])), float(np.mean(midpoint[:, 1])))

        else:
            self.patience_cross_frame=0
            left_y = np.float32(150)
            left_x = np.float32(self.left_fit[0] * left_y ** 2 + self.left_fit[1] * left_y + self.left_fit[2])
            right_y = np.float32(150)
            right_x = np.float32(self.right_fit[0] * right_y ** 2 +self.right_fit[1] * right_y + self.right_fit[2])
            midpoint = np.asarray(self.midpoint)
            cte = self.calcultate_cte(float(np.mean(midpoint[:, 0])), float(np.mean(midpoint[:, 1])))

        if len(bounding_cars)>0 or (self.patience_cars_frame<patience_car_thread and self.cars is not None):
            if len(bounding_cars)>0:
                print 'cars detect'
                self.patience_cars_frame=0
                self.cars=bounding_cars
            elif self.patience_cars_frame<patience_car_thread and self.cars is not None:
                print 'patience '
                self.patience_cars_frame+=1
            x, y, w, h = self.cars[0]
            interset_point = (x + w / 2, y + h / 2)
            if confirmlr=='left':
                self.cars.append(True)
                if lre=='left':
                    right_x = int(2 * right_x - left_x)
                    right_y = int(2 * right_y - left_y)

            elif confirmlr=='right':
                self.cars.append(False)
                if lre=='right':
                    left_x = int(2 * left_x - right_x)
                    left_y = int(2 * left_y - right_y)


            elif lre=='left':
                self.cars.append(True)
                right_x=int(2*right_x-left_x)
                right_y=int(2*right_y-left_y)
            elif lre=='right':
                self.cars.append(False)
                left_x=int(2*left_x-right_x)
                left_y=int(2*left_y-right_y)
            else:
                self.cars.append(interset_point[0] - list_left_x[-1] < list_right_x[-1] - interset_point[0])

            is_left_bias = self.cars[1]
            print 'detect_left' if is_left_bias else 'detect_right'


            if self.cars[1]: #is_left_bias
                left_x = x + int(0.9*w)
                left_y = y +int(0.9*h)
                cte = self.calcultate_cte(float(left_x + right_x) / 2, float(left_y + right_y) / 2)
            else:
                right_x = x
                right_y = y + int(0.9*h)
                cte = self.calcultate_cte(float(left_x + right_x) / 2, float(left_y + right_y) / 2)

        if len(sign_directions)==0:
            self.cross_history.append('no')
        else:
            for sign_direction in sign_directions:
                self.cross_history.append(sign_direction)
        self.cross_history=self.cross_history[-10:]
        num_left_turn=0
        num_right_turn=0
        for cross in self.cross_history:
            if cross is 'sign_detect_left':
                num_left_turn+=1
            elif cross is 'sign_detect_right':
                num_right_turn+=1
        if Is_cross:
            if num_left_turn>num_right_turn and num_left_turn+num_right_turn>=3:
                cte=-40
            elif num_right_turn>num_left_turn and num_right_turn+num_left_turn>=3:
                cte=40
        else:
            if num_left_turn>num_right_turn and num_left_turn+num_right_turn>=5:
                cte=-40
            elif num_right_turn>num_left_turn and num_right_turn+num_left_turn>=5:
                cte=40


        pid_cte=self.PID(cte)
        self.pre_cte = pid_cte
        speed=self.Get_Speed_By_Cte(pid_cte)
        real_speed=self.get_Current_speed(speed) if self.patience_cross_frame==patience_cross_thread else 20
        mid_x,mid_y=np.float32((left_x + right_x) / 2), np.float32((left_y + right_y) / 2)
        if len(self.midpoint)< 10 and self.patience_cross_frame==patience_cross_thread:
            self.midpoint.append([mid_x,mid_y])
        elif self.patience_cross_frame==patience_cross_thread:
            del self.midpoint[0]
            self.midpoint.append([mid_x,mid_y])

        return pid_cte,real_speed,left_x,left_y,right_x,right_y,mid_x,mid_y

    def get_midpont_fit(self):
        midpoint=np.asarray(self.midpoint)
        return np.polyfit(midpoint[:,1],midpoint[:,0],2)


















