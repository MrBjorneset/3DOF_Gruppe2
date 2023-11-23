import multiprocessing as mp
import numpy as np
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
import control as ctrl
from tkinter import *

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

class PIDController:
    def __init__(self, kp, ki, kd, setpoint, sample_time):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self.prev_error = 0
        self.integral = 0

    def update(self, current_value):
        error = self.setpoint - current_value

        self.integral += error * self.sample_time
        derivative = (error - self.prev_error) / self.sample_time

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error

        return float(output)

pid_controller_x = PIDController(kp=10, ki=1, kd=0.02, setpoint=7, sample_time=0.1)
pid_controller_y = PIDController(kp=7, ki=1, kd=0.02, setpoint=2, sample_time=0.1)

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 90
servo1_angle_limit_negative = -90

servo2_angle_limit_positive = 90
servo2_angle_limit_negative = -90

servo3_angle_limit_positive = 90
servo3_angle_limit_negative = -90


def ball_track(key1, queue):
    camera_port = 0
    cap = cv2.VideoCapture(camera_port,cv2.CAP_DSHOW)
    cap.set(3, 1280)
    cap.set(4, 720)

    get, img = cap.read()
    h, w, _ = img.shape

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 0, 'smin': 52, 'vmin': 187, 'hmax': 9, 'smax': 255, 'vmax': 238}

    center_point = [626, 337, 2210] # center point of the plate, calibrated
    

    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)
        
        if countours:

            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                   round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                   round(int(countours[0]['area'] - center_point[2])/100)

        
            queue.put(data)
            #print("The got coordinates for the ball are :", data)
        else:
            data = 'nil' # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)


def servo_control(key2, queue):
    port_id = 'COM3'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    
    if key2:
        print('Servo controls are initiated')


    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = math.radians(float(angle_passed1))
        servo2_angle = math.radians(float(angle_passed2))
        servo3_angle = math.radians(float(angle_passed3))
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def writeCoord():
        """
        Here in this function we get both coordinate and servo control, it is an ideal place to implement the controller
        """
        corrd_info = queue.get()
        def constrain(lower, val, upper):
            return max(lower, max(val, upper))
        
        def ikine(p, r):
            R = 40
            L = 225
            p = p * np.pi/180
            r = r * np.pi/180
            z = np.array([  np.sqrt(3)*L/6 * np.sin(p)*np.cos(r) - L/2*np.sin(r),
                        np.sqrt(3)*L/6 * np.sin(p)*np.cos(r) + L/2*np.sin(r),
                        -np.sqrt(3)*L/6 * np.sin(p)*np.cos(r)])
            zR = np.zeros(3)
            for i in range(3):
                zR[i] = constrain(-1, z[i]/R, 1)
                Va = np.arcsin(z/R) * 180/np.pi
            return Va


        if corrd_info == 'nil': # Checks if the output is nil
            print('cant find the ball :(')
        else:
            """
            Pservo = np.array([[-14, 14, 0],
                  [8, -16, 0],
                  [24, 18,0]])
        
            vp = corrd_info[0]
            Tp = np.array([[1, 0, 0],
                 [0,np.cos(vp), -np.sin(vp)],
                 [0, np.sin(vp), np.cos(vp)]])

            Pp = np.multiply(Pservo, Tp)

            vr = corrd_info[1]
            Tr = np.array([[np.cos(vr), 0, -np.sin(vr)],
                [0, 1, 0],
                [np.sin(vr), 0, np.cos(vr)]])

            Ppr = np.multiply(Pp, Tr)

            zmatrise = np.array([[0, 0, 0],
                [0,0,0],
                [corrd_info[2], corrd_info[2], corrd_info[2]]])
            
            Pprz = np.add(Ppr, zmatrise)

            
            print(Ppr)
            print(Pprz)
            """
            print('The position of the ball : ', corrd_info)
            Va = ikine(corrd_info[0], corrd_info[1])
            if (-24 < corrd_info[0] < 34) and (-22 < corrd_info[1] < 34) and (-90 < corrd_info[2] < 90) and (servo1_angle_limit_negative < servo1_angle < servo1_angle_limit_positive) and (servo2_angle_limit_negative < servo2_angle < servo2_angle_limit_positive) and (servo3_angle_limit_negative < servo3_angle < servo3_angle_limit_positive):

                all_angle_assign(0,0,0)
            else:
                all_angle_assign(0,0,0)

    def write_arduino(data):
        print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():
        ang1 = servo1_angle
        ang2 = servo2_angle
        ang3 = servo3_angle

        angles: tuple = (round(math.degrees(ang1), 1),
                         round(math.degrees(ang2), 1),
                         round(math.degrees(ang3), 1))

        write_arduino(str(angles))

    while key2 == 2:
        writeCoord()

    root.mainloop()  # running loop

if __name__ == '__main__':

    queue = Queue() # The queue is done inorder for the communication between the two processes.
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target= ball_track, args=(key1, queue)) # initiate ball tracking process
    p2 = mp.Process(target= servo_control,args=(key2, queue)) # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()