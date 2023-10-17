#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
* @par Copyright (C): 2010-2020, hunan CLB Tech
* @file         48_QR_code.py
* @version      V2.0
* @details
* @par History

@author: zhulin
"""
# import the necessary packages
from imutils.video import VideoStream
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import Adafruit_PCA9685
import time
import cv2

import sys



#初始化PCA9685和舵机
#servo_pwm = Adafruit_PCA9685.PCA9685()  # 实例话舵机云台

# 设置舵机初始值，可以根据自己的要求调试
#servo_pwm.set_pwm_freq(60)  # 设置频率为60HZ
#servo_pwm.set_pwm(5,0,325)  # 底座舵机
#servo_pwm.set_pwm(4,0,325)  # 倾斜舵机







# loop over the frames from the video stream
def readQR():
	print("[INFO] starting QR reading")
	vs = cv2.VideoCapture(0) # cv video capture
	time.sleep(0.5)
	
	
    # loop over the detected barcodes
	INFO = None
	while(INFO is None):
		_, frame = vs.read()
		frame = imutils.resize(frame, width=400)
		# find the barcodes in the frame and decode each of the barcodes
		barcodes = pyzbar.decode(frame)
		for barcode in barcodes:
			INFO = int(barcode.data)
			print("barcode: ",INFO)
			print("barcode type", type(INFO))
	
		# extract the bounding box location of the barcode and draw
		# the bounding box surrounding the barcode on the image
		
	return INFO
    # # show the output frame
	# cv2.imshow("Barcode Scanner", frame)
	# key = cv2.waitKey(1) & 0xFF
 
	# # if the `q` key was pressed, break from the loop
	# if key == ord("q"):
	# 	# close the output CSV file do a bit of cleanup
	# 	print("[INFO] cleaning up...")
	# 	cv2.destroyAllWindows()
	# 	return None
	

if __name__== "__main__":
	while(True):
		info = readQR()
		print("INFO READ : ",info)
	