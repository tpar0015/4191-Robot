import cv2
import numpy as np
import time
import multiprocessing as mp
"""
Author @ Hanlin XUUUU
using cv2 HoughCircles to detect image circles
"""
def colorCircles():
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
        hmn_r = 165
        hmx_r = 179
        smn_r = 118
        smx_r = 255
        vmn_r = 189
        vmx_r = 220

        hmn_b = 98
        hmx_b = 140
        smn_b = 118
        smx_b = 255
        vmn_b = 42
        vmx_b = 100

        hmn_g = 57
        hmx_g = 88
        smn_g = 110
        smx_g = 255
        vmn_g = 57
        vmx_g = 146
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
        circles_red = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,3,160,param1=120,param2=70,minRadius=40,maxRadius=80)
        

        # filter blue
        dilation = cv2.dilate(tracking_blue,kernel,iterations = 1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        closing = cv2.GaussianBlur(closing,(5,5),0)
        circles_blue = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,3,160,param1=120,param2=70,minRadius=40,maxRadius=80)
        cv2.imshow('closing',closing)
        # filter green
        dilation = cv2.dilate(tracking_green,kernel,iterations = 1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        closing = cv2.GaussianBlur(closing,(5,5),0)
        circles_green = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,3,160,param1=120,param2=70,minRadius=40,maxRadius=80)




        # color detected and draw iin the window
        if circles_red is not None:
            print("red")
            x, y, r = circles_red[0][0]
            x_p = int(round(x))
    
            for i in circles_red[0,:]:
                    
                    if int(round(i[2])) < 30:
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,255,0),10)
                    
                    elif int(round(i[2])) > 35:
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
        elif circles_blue is not None:
            print("blue")
            x, y, r = circles_blue[0][0]
            for i in circles_blue[0,:]:
            
                    if int(round(i[2])) < 30:
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,255,0),10)
                    
                    elif int(round(i[2])) > 35:
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
        elif circles_green is not None:
            print("green")
            x, y, r = circles_green[0][0]
            for i in circles_green[0,:]:
            
                    if int(round(i[2])) < 30:
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,255,0),10)
                    
                    elif int(round(i[2])) > 35:
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
                            cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
        else:
            print("None")
        
        cv2.imshow('tracking',frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows() 
                break

    cap.release()
    cv2.destroyAllWindows() 
def test():
    print("test")

if __name__ == '__main__':
     p1 = mp.Process(target=colorCircles,args = ())
     p2 = mp.Process(target=test,args = ())
     p1.start()
     p2.start()
