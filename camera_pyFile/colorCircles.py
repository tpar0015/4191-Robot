import cv2
import numpy as np
import time  
"""
Author @ Hanlin XUUUU
using cv2 HoughCircles to detect image circles
"""
# kernal cap init
kernel = np.ones((5,5),np.uint8) # kernal is used for filter 
cap = cv2.VideoCapture(0) # cv video capture

time.sleep(0.5)

# window size default change 
cap.set(3,320)
cap.set(4,240)


# windows

cv2.namedWindow('closing')
cv2.namedWindow('tracking')




while(1):
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



    # color detected and draw iin the window
    if circles_red is not None:
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
         print("blue")
    elif circles_green is not None:
         print("green")
    else:
        print("None")
    #cv2.imshow('closing',closing)
    #cv2.imshow('tracking',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows() 
            break

cap.release()
cv2.destroyAllWindows() 