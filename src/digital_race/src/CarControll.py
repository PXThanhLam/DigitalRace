import cv2
#from LaneDetection import pipeline
import numpy as np
import math

pi = 3.141592654
patience_thread=5
class CarControll:
    def __init__(self):
        self.x_car=160
        self.y_car=240
        self.left_fit=None
        self.right_fit=None

        self.t_kP=0.9
        self.t_kI=0.001
        self.t_kD=0.1
        self.pre_cte=0

        self.P=0
        self.I=0
        self.D=0

        self.is_slowDown=False
        self.current_velocity=40
        self.patienceframe=0


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
            if self.patienceframe<=patience_thread:
                self.patienceframe+=1
            else :
                self.patienceframe=0
                self.current_velocity=speed
        else:
            self.patienceframe=0
            self.current_velocity=speed
        return self.current_velocity




    def PID(self,cte):
        self.P=cte
        self.I+=cte
        self.D=cte-self.pre_cte
        pid_cte=self.t_kP*self.P +self.t_kD*self.D +self.I*self.t_kI
        self.pre_cte=cte
        if pid_cte>60:
            pid_cte=60
        if pid_cte<-60:
            pid_cte=-60

        return pid_cte


    def driveCar(self,left_x,left_y,right_x,right_y,left_fit,right_fit,velocity):
        if left_fit is not None:
            self.left_fit = left_fit
        if right_fit is not None:
            self.right_fit = right_fit
        left_num=len(left_x)//2 if len(left_x) >0 else 0
        right_num=len(right_x)//2 if len(right_x)>0 else 0

        cte=self.pre_cte

        if(left_num>0 and right_num>0):
            left_x = left_x[left_num - 1]
            left_y = left_y[left_num - 1]
            right_x = right_x[right_num - 1]
            right_y = right_y[right_num - 1]
            cte=self.calcultate_cte(float(left_x+right_x)/2,float(left_y+right_y)/2)

        elif left_num >0:
            left_x = left_x[left_num - 1]
            left_y = left_y[left_num - 1]
            right_y = np.float32(140)
            right_x=np.float32(self.right_fit[0] * right_y** 2 + self.right_fit[1] * right_y+ self.right_fit[2])
            cte = self.calcultate_cte(float(left_x + right_x) / 2, float(left_y + right_y) / 2)

        elif right_num>0:
            right_x = right_x[right_num - 1]
            right_y = right_y[right_num - 1]
            left_y = np.float32(140)
            left_x = np.float32(self.left_fit[0] * left_y ** 2 +self.left_fit[1] * left_y +self.left_fit[2])
            cte = self.calcultate_cte(float(left_x + right_x) / 2, float(left_y + right_y) / 2)
        else:

            left_y = np.float32(140)
            left_x = np.float32(self.left_fit[0] * left_y ** 2 + self.left_fit[1] * left_y + self.left_fit[2])
            right_y = np.float32(140)
            right_x = np.float32(self.right_fit[0] * right_y ** 2 +self.right_fit[1] * right_y + self.right_fit[2])
            cte = self.calcultate_cte(float(left_x + right_x) / 2, float(left_y + right_y) / 2)

        pid_cte=self.PID(cte)
        speed=self.Get_Speed_By_Cte(pid_cte)
        real_speed=self.get_Current_speed(speed)

        return pid_cte,real_speed,left_x,left_y,right_x,right_y,np.float32((left_x + right_x) / 2), np.float32((left_y + right_y) / 2)


















