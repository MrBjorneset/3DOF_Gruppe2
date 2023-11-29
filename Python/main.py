import multiprocessing as mp
import numpy as np
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
import time
import control as ctrl
from tkinter import *

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""
class PIDController:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.previous_error = 0
        self.integral = 0
        self.windup = 20
        self.last_time = time.time()

    def compute(self, setpoint, actual_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        
        error = setpoint - actual_value
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error

        if (self.integral > self.windup):
            self.integral = -self.windup
        if (self.integral < -self.windup):
            self.integral = self.windup
        
        self.last_time = current_time

        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

Kp = 0.3
Ki = 0.05
Kd = 0.15

PID_X = PIDController(Kp, Ki , Kd)
PID_Y = PIDController(Kp, Ki , Kd)

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
lim = 30
servo1_angle_limit_positive = lim
servo1_angle_limit_negative = -lim

servo2_angle_limit_positive = lim
servo2_angle_limit_negative = -lim

servo3_angle_limit_positive = lim
servo3_angle_limit_negative = -lim


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
    hsvVals = {'hmin': 0, 'smin': 19, 'vmin': 214, 'hmax': 9, 'smax': 125, 'vmax': 255}

    center_point = [626, 337, 2210] # center point of the plate, calibrated
    

    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask, minArea=2000, maxArea=5500)
        
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
        crosshair_size = 20
        crosshair_thickness = 2
        crosshair_color = (255, 255, 255)  # white color

        # Draw vertical line (plus sign)
        cv2.line(imgStack, (w // 2, h // 2 - crosshair_size), (w // 2, h // 2 + crosshair_size), crosshair_color, crosshair_thickness)

        # Draw horizontal line (plus sign)
        cv2.line(imgStack, (w // 2 - crosshair_size, h // 2), (w // 2 + crosshair_size, h // 2), crosshair_color, crosshair_thickness)

        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)


def incline(p, r) :
        R = 4
        L = 22.5
        p = p * np.pi/180 * 20
        r = r * np.pi/180 * 20
        
        # Define the position array
        PosMatNor = np.array([
                            (L/2, L/2*np.sqrt(3), 0),
                            (-L/2, L/2*np.sqrt(3), 0),
                            (0, -L/np.sqrt(3), 0)
                            ])

        # Define the rotation angle in radians
        v = np.radians(300) # Adjust the angle as needed

        # Define the rotation matrix for a rotation around the z-axis
        rotation_matrix = np.array([
                                    [np.cos(v), -np.sin(v), 0],
                                    [np.sin(v), np.cos(v), 0],
                                    [0, 0, 1]
                                    ])

        # Apply the rotation to the position array
        PosMatRotated = np.dot(PosMatNor, rotation_matrix)

        #Getting angle for pitch and roll for PID and chaning to radians.
        Vp = p * np.pi / 180
        Vr = r * np.pi / 180
                    
        Transform_matrix_p = np.array ([(1, 0, 0),
                                        (0, np.cos(Vp),-np.sin(Vp)),
                                        (0, np.sin(Vp),np.cos(Vp))
                                        ])
                    
        PosPitch = np.dot(PosMatRotated, Transform_matrix_p)

        Transform_matrix_r = np.array ([(np.cos(Vr), 0, -np.sin(Vr)),
                                        (0, 1, 0),
                                        (np.sin(Vr), 0, np.cos(Vr))
                                        ])
                    
        z = np.dot(PosPitch,Transform_matrix_r)
        Va = np.zeros(3)
        Va[0] = np.arcsin(z[0][2]/R) * 180/np.pi
        Va[1] = np.arcsin(z[1][2]/R) * 180/np.pi
        Va[2] = np.arcsin(z[2][2]/R) * 180/np.pi

        return Va

def servo_control(key2, queue):
    port_id = 'COM3'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    
    if key2:
        print('Servo controls are initiated')


    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = -math.radians(float(angle_passed1))
        servo2_angle = -math.radians(float(angle_passed2))
        servo3_angle = -math.radians(float(angle_passed3))
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def writeCoord():
        """
        Here in this function we get both coordinate and servo control, it is an ideal place to implement the controller
        """
        corrd_info = queue.get()
        
        try:
            float_array = [float(value) for value in corrd_info]
            ball_x = PID_X.compute(8.0, float_array[0])
            ball_y = PID_Y.compute(-8.0, float_array[1])
            Va = incline(ball_x, ball_y) # Endre ball_x og ball_y til Output ifrå PID for x og y
        except ValueError:
            print('Invalid coordinate values:', corrd_info)
            return  # Skip the rest of the function if the conversion fails
        

        print('The position of the ball : ', corrd_info)

        if (-34 < corrd_info[0] < 44) and (-34 < corrd_info[1] < 44) and (-90 < corrd_info[2] < 90) and (
            servo1_angle_limit_negative < servo1_angle < servo1_angle_limit_positive) and (
            servo2_angle_limit_negative < servo2_angle < servo2_angle_limit_positive) and (
            servo3_angle_limit_negative < servo3_angle < servo3_angle_limit_positive):
            
            all_angle_assign(Va[0], Va[1], Va[2])
            
        else:
            all_angle_assign(0, 0, 0)

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