from picamera import PiCamera
from time import sleep

def capture(time, path):
    
    camera = PiCamera()
    sleep(time)
    #camera.capture('/home/hanlinpi/Desktop/Project_4191/camera4.jpg')
    camera.capture(path)