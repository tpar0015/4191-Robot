from time import sleep
from picamera import PiCamera
import cv2

filename = "foo.jpg"

camera = PiCamera()
camera.resolution = (2592, 1944) # check version of NoIR camera
camera.start_preview()
# Camera warm-up time
sleep(2)

# Capture a photo
camera.capture(filename)


def localise():
    
    # check camera
    # find corners of arena
    return x_pos, y_pos, x_target, y_target

def aim(x_pos, y_pos, x_target, y_target):
    # turn so that launcher faces target
    # check pose relative to corners
    pass