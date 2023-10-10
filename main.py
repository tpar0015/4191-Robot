## Main running loop for the package delivery robot
## Implements a state machine with two states: 'Load' and 'Fire'
from electromagnet import Electromagnet
from MotorControl.motorctl_new import Motor
from MotorControl.rotary_new import RotaryEncoder
from MotorControl.drive_test_final import Drive
from multiprocessing import Queue, Process
from pins import *
from load import load
from time import sleep
from aiming import aiming, turning_angle
import numpy as np

"""
Main Procedure:
1. Load Robot 
2. Put colour in front to detect Colour
3. Use colour to determine pullback and aiming angle
4. Use motor and electromagnet to drawback
5. Pivot to angle
6. Shoot
7. Repeat from 1
"""

def aim_shoot(robot_pose, color_sticker="red"):
    ##! Load cycle
    #input("Continue to run load")
    load(mag, launcher)
    
    ##! Fire cycle
    # Orient toward the target
    motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
    test_queue = Queue()
    robot_control = Drive(robot_pose, test_queue)
    
    input("Continue to run aiming")
    aiming(mag, motor, robot_control, robot_pose, color_sticker)

    # Reset
    # Change current pose depending on sticker
    robot_pose = [0,0,color_sticker]
    sleep(10)



if __name__== "__main__":
    # initialise electromagnet, electromagnet drawback motor
    mag = Electromagnet()
    launcher = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)

    # initialise wheels
    #wheel_left = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)
    #wheel_right = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"], speed=100)
    encoder_left = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"])
    encoder_right = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"])

    # Package 1 (smallest): Red
    # Package 2 (middle): Green
    # Package 3 (large): Blue

    """
    Aiming
    robot_pose = -1(left), 0(center), 1(right)
    color_sticker = -1(red), 0(blue), 1(green)
    color_sticker - robot_pose = 0 (don't turn), < 0 (turn right), > 0 (turn left)
    """
    robot_pose = [0,0,0]

    ### Without colour detector, reload and shoot in middle
    while True:
        # Check current colour for destination
        color_sticker = "red"#get colour output
        ##! Load cycle
        #input("Continue to run load")
        load(mag, launcher)
        
        
        ##! Fire cycle
        # Orient toward the target
        motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
        test_queue = Queue()
        robot_control = Drive(robot_pose, test_queue)
        
        input("Continue to run aiming")
        aiming(mag, motor, robot_control, robot_pose, color_sticker)

        # Reset
        # Change current pose depending on sticker
        robot_pose = [0,0,color_sticker]
        sleep(10)
    


'''
    ### With Colour Detection
    kernel = np.ones((5,5),np.uint8) # kernal is used for filter 
    cap = cv2.VideoCapture(0) # cv video capture
    sleep(0.5)

    # window size default change 
    cap.set(3,320)
    cap.set(4,240)
    # windows
    cv2.namedWindow('closing')
    cv2.namedWindow('tracking')

    while True:
        _, frame = cap.read()

        # HSV
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hue,sat,val = cv2.split(hsv)

        # get red color info
        # In HVS red blue and green is 3 distinct color and ease to distinguish colors
        hmn_r = 0
        hmx_r = 10
        smn_r = 50
        smx_r = 255
        vmn_r = 50
        vmx_r = 255

        hmn_b = 100
        hmx_b = 140
        smn_b = 50
        smx_b = 255
        vmn_b = 50
        vmx_b = 255

        hmn_g = 40
        hmx_g = 80
        smn_g = 50
        smx_g = 255
        vmn_g = 50
        vmx_g = 255
        # RED

        # apply and check if frame is in the range
        hthresh_red = cv2.inRange(np.array(hue),np.array(hmn_r),np.array(hmx_r))
        sthresh_red = cv2.inRange(np.array(sat),np.array(smn_r),np.array(smx_r))
        vthresh_red = cv2.inRange(np.array(val),np.array(vmn_r),np.array(vmx_r))

        # using bitwise and to combine all filterd result
        tracking_red = cv2.bitwise_and(hthresh_red,cv2.bitwise_and(sthresh_red,vthresh_red)) 

        # Blue  
        hthresh_blue = cv2.inRange(np.array(hue),np.array(hmn_b),np.array(hmx_b))
        sthresh_blue = cv2.inRange(np.array(sat),np.array(smn_b),np.array(smx_b))
        vthresh_blue = cv2.inRange(np.array(val),np.array(vmn_b),np.array(vmx_b))

        # using bitwise and to combine all filterd result
        tracking_blue = cv2.bitwise_and(hthresh_blue,cv2.bitwise_and(sthresh_blue,vthresh_blue)) 

        # Green
        hthresh_green = cv2.inRange(np.array(hue),np.array(hmn_g),np.array(hmx_g))
        sthresh_green = cv2.inRange(np.array(sat),np.array(smn_g),np.array(smx_g))
        vthresh_green = cv2.inRange(np.array(val),np.array(vmn_g),np.array(vmx_g))

        # using bitwise and to combine all filterd result
        tracking_green = cv2.bitwise_and(hthresh_green,cv2.bitwise_and(sthresh_green,vthresh_green)) 

        # filter red
        dilation = cv2.dilate(tracking_red,kernel,iterations = 1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        closing = cv2.GaussianBlur(closing,(5,5),0)
        circles_red = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,2,150,param1=150,param2=70,minRadius=20,maxRadius=60)
        

        # filter blue
        dilation = cv2.dilate(tracking_blue,kernel,iterations = 1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        closing = cv2.GaussianBlur(closing,(5,5),0)
        circles_blue = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,2,150,param1=150,param2=70,minRadius=20,maxRadius=60)

        # filter green
        dilation = cv2.dilate(tracking_green,kernel,iterations = 1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        closing = cv2.GaussianBlur(closing,(5,5),0)
        circles_green = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,2,150,param1=150,param2=70,minRadius=20,maxRadius=60)


        detect_color = None
        # color detected and draw iin the window
        if circles_red is not None:
            detect_color = "red"
            print("red")
            # x, y, r = circles_red[0][0]
            # x_p = int(round(x))
    
            # for i in circles_red[0,:]:
                    
            #         if int(round(i[2])) < 30:
            #                 cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
            #                 cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,255,0),10)
                    
            #         elif int(round(i[2])) > 35:
            #                 cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
            #                 cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
        elif circles_blue is not None:
            detect_color = "blue"
            print("blue")
        elif circles_green is not None:
            detect_color = "green"
            print("green")
        else:
            print("None")
        #cv2.imshow('closing',closing)
        cv2.imshow('tracking',frame)

        if detect_color != None:
            aim_shoot(robot_pose, color_sticker=detect_color)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows() 
            break

    cap.release()
    cv2.destroyAllWindows() 
'''