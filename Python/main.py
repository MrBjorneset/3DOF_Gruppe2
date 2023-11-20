import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
from tkinter import *

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""
# Create a PID controller class
class PIDController:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# Initialize PID controllers for each servo angle
pid1 = PIDController(P=1, I=0.1, D=0.0)
pid2 = PIDController(P=1, I=0.1, D=0.0)
pid3 = PIDController(P=1, I=0.1, D=0.0)

# define servo angles and set a value
servo1_angle = -4
servo2_angle = -9
servo3_angle = -6
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 90
servo1_angle_limit_negative = -90

servo2_angle_limit_positive = 90
servo2_angle_limit_negative = -90

servo3_angle_limit_positive = 90
servo3_angle_limit_negative = -90

# -------------------------------------------Ball Tracker-------------------------------------------

def ball_track(key1, queue):
    camera_port = 1
    cap = cv2.VideoCapture(camera_port,cv2.CAP_DSHOW)
    cap.set(3, 1280)
    cap.set(4, 720)

    get, img = cap.read()
    h, w, _ = img.shape

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(FALSE)  # if you want to find the color and calibrate the program we use this *(Debugging)
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
            print("The got coordinates for the ball are :", data)
        else:
            data = 'nil' # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        #imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)

# -------------------------------------------Servo Control-------------------------------------------

def servo_control(key2, queue):
    port_id = 'COM3'     # endre com porten til arduinoen etter behov
    # initialise serial interface
    arduino = serial.Serial(port_id, 250000, timeout=0.1)
    dt = 0.1 # time step
    if key2:
        print('Servo controls are initiated')

# Assign new angles to the servos
    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        target_angle1 = math.radians(float(angle_passed1))
        target_angle2 = math.radians(float(angle_passed2))
        target_angle3 = math.radians(float(angle_passed3))

        servo1_angle += pid1.update(target_angle1 - servo1_angle, dt)
        servo2_angle += pid2.update(target_angle2 - servo2_angle, dt)
        servo3_angle += pid3.update(target_angle3 - servo3_angle, dt)

        write_servo()

    root = Tk()
    root.resizable(0, 0)


    def writeCoord():
        corrd_info = queue.get()

        if corrd_info == 'nil': # Checks if the output is nil
            print('cant find the ball :(')
        else:
            print('The position of the ball : ', corrd_info[2])

            if (servo1_angle_limit_negative < corrd_info[0] < servo1_angle_limit_positive) and (servo2_angle_limit_negative < corrd_info[1] < servo2_angle_limit_positive) and (servo3_angle_limit_negative < corrd_info[2] < servo3_angle_limit_positive):

                all_angle_assign(corrd_info[0],corrd_info[1],corrd_info[2])
            else:
                all_angle_assign(-4,-9,-6)

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

    while key2:
        corrd_info = queue.get()
            
    root.mainloop()  # running loop

if __name__ == '__main__':

    queue = Queue() # The queue is done inorder for the communication between the two processes.
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target= ball_track, args=(key1, queue)) # initiate ball tracking process
    p2 = mp.Process(target=servo_control,args=(key2, queue)) # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()