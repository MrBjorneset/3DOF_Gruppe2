import multiprocessing as mp
import numpy as np
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import pandas as pd
import time
import control as ctrl
from tkinter import *

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
start_time = time.time()

def pid(P, I, D, setpoint, actual_value, previous_error, integral, windup, last_time):
    current_time = time.time()
    delta_time = current_time - last_time
    
    error = setpoint - actual_value
    integral += error * delta_time
    derivative = (error - previous_error) / delta_time
    previous_error = error

    if integral > windup:
        integral = -windup
    elif integral < -windup:
        integral = windup
    
    last_time = current_time
    print('The PID values are : ', P * error, I * integral, D * derivative)
    return P * error + I * integral + D * derivative, previous_error, integral, last_time


# Usage in your code
Kp = 3.5
Ki = 0
Kd = 0

PID_X = 0
PID_Y = 0
previous_error_x = 0
integral_x = 0
last_time_x = time.time()

previous_error_y = 0
integral_y = 0
last_time_y = time.time()


pid_data = []

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
    hsvVals = {'hmin': 0, 'smin': 124, 'vmin': 216, 'hmax': 39, 'smax': 255, 'vmax': 255}

    center_point = [626, 337, 2210] # center point of the plate, calibrated
    

    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask, minArea=1000, maxArea=5000)
        
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


def incline(pitch, roll):
    R = 40
    L = 225

    pitch_rad = pitch * np.pi / 180 
    roll_rad  = roll  * np.pi / 180 

    # Define the position array
    pos_mat_nor = np.array([
        [L / 2, L / 2 * np.sqrt(3), 0],
        [-L / 2, L / 2 * np.sqrt(3), 0],
        [0, -L / np.sqrt(3), 0]
    ])

    # Define the rotation angle in radians
    rotation_angle = np.radians(280)

    # Define the rotation matrix for a rotation around the z-axis
    rotation_matrix = np.array([
        [np.cos(rotation_angle), -np.sin(rotation_angle), 0],
        [np.sin(rotation_angle), np.cos(rotation_angle), 0],
        [0, 0, 1]
    ])

    # Apply the rotation to the position array
    pos_mat_rotated = np.dot(pos_mat_nor, rotation_matrix)

    # Transformations for pitch
    transform_matrix_pitch = np.array([
        [1, 0, 0],
        [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        [0, np.sin(pitch_rad), np.cos(pitch_rad)]
    ])

    pos_pitch = np.dot(pos_mat_rotated, transform_matrix_pitch)

    # Transformations for roll
    transform_matrix_roll = np.array([
        [np.cos(roll_rad), 0, -np.sin(roll_rad)],
        [0, 1, 0],
        [np.sin(roll_rad), 0, np.cos(roll_rad)]
    ])

    z = np.dot(pos_pitch, transform_matrix_roll)

    # Calculate angles
    va = np.arcsin(z[:, 2] / R) * 180 / np.pi

    return va

def servo_control(key2, queue):
    port_id = 'COM3'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    
    if key2:
        print('Servo controls are initiated')
    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = -(float(angle_passed1))
        servo2_angle = -(float(angle_passed2))
        servo3_angle = -(float(angle_passed3))
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def writeCoord():
        global PID_X, PID_Y, previous_error_x, integral_x, last_time_x, previous_error_y, integral_y, last_time_y
        corrd_info = queue.get()
        
        try:
            float_array = [float(value) for value in corrd_info]
        except ValueError:
            #print('Invalid coordinate values:', corrd_info)
            return  # Skip the rest of the function if the conversion fails

        print('The position of the ball : ', corrd_info)
        

        if (-34 < corrd_info[0] < 44) and (-34 < corrd_info[1] < 44) and (-9000 < corrd_info[2] < 9000) and (
            servo1_angle_limit_negative < servo1_angle < servo1_angle_limit_positive) and (
            servo2_angle_limit_negative < servo2_angle < servo2_angle_limit_positive) and (
            servo3_angle_limit_negative < servo3_angle < servo3_angle_limit_positive):

                    # Use the new pid function for Roll and Pitch
            Roll, previous_error_x, integral_x, last_time_x =  pid(Kp, Ki, Kd, 6, float_array[1], previous_error_x, integral_x, 200, last_time_x)
            Pitch, previous_error_y, integral_y, last_time_y = pid(Kp, Ki, Kd, 1, float_array[0], previous_error_y, integral_y, 200, last_time_y)

            current_time = time.time() - start_time
            
            pid_data.append({'Time': current_time, 'Output_X': float_array[0], 'Output_Y': float_array[1]})
            ContAng = incline(float_array[0], float_array[1])
            all_angle_assign(ContAng[0], ContAng[1], ContAng[2])
            
        else:
            all_angle_assign(0, 0, 0)

        
    def write_arduino(data):
        print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():
        ang1 = servo1_angle
        ang2 = servo2_angle
        ang3 = servo3_angle

        angles: tuple = (round((ang1), 1),
                         round((ang2), 1),
                         round((ang3), 1))

        write_arduino(str(angles))

    while key2 == 2:
        global current_time
        current_time = time.time() - start_time
        writeCoord()
        if current_time % 0.5 < 0.1:  # Change the interval to 0.5 seconds
            df = pd.DataFrame(pid_data)
            df.to_excel('pid_data.xlsx', index=False)

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