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
    camera_port = 0
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
    if key2:
        print('Servo controls are initiated')
    
    # PID controllers for each servo
    pid1 = {'Kp': 0.1, 'Ki': 0.01, 'Kd': 0.05, 'dt': 0.1, 'integral': 0, 'prev_error': 0}
    pid2 = {'Kp': 0.2, 'Ki': 0.02, 'Kd': 0.1, 'dt': 0.1, 'integral': 0, 'prev_error': 0}
    pid3 = {'Kp': 0.3, 'Ki': 0.03, 'Kd': 0.15, 'dt': 0.1, 'integral': 0, 'prev_error': 0}

    def pid_controller(setpoint, current_value, pid_params):
        Kp, Ki, Kd, dt = pid_params['Kp'], pid_params['Ki'], pid_params['Kd'], pid_params['dt']
    
        error = setpoint - current_value
        pid_params['integral'] += error * dt
        derivative = (error - pid_params['prev_error']) / dt
    
        output = Kp * error + Ki * pid_params['integral'] + Kd * derivative
    
        pid_params['prev_error'] = error
    
        return output


# Assign new angles to the servos
    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        target_angle1 = math.radians(float(angle_passed1))
        target_angle2 = math.radians(float(angle_passed2))
        target_angle3 = math.radians(float(angle_passed3))

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

        if corrd_info == 'nil':
            print('Cannot find the ball :(')
            all_angle_assign(-4, -9, -6)
        else:
            print('The position of the ball:', corrd_info[2])

            if (servo1_angle_limit_negative < corrd_info[0] < servo1_angle_limit_positive) and \
                    (servo2_angle_limit_negative < corrd_info[1] < servo2_angle_limit_positive) and \
                    (servo3_angle_limit_negative < corrd_info[2] < servo3_angle_limit_positive):

                # Use PID controllers to adjust servo angles
                new_angle1 = pid_controller(0, servo1_angle, pid1)
                new_angle2 = pid_controller(0, servo2_angle, pid2)
                new_angle3 = pid_controller(0, servo3_angle, pid3)

                all_angle_assign(new_angle1, new_angle2, new_angle3)
            
                

        root.update()  # Update the Tkinter window
        root.after(int(0.01 * 1000), root.quit)  # Wait for dt seconds and then close the Tkinter window

    root.mainloop()

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