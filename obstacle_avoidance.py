from gpiozero import DistanceSensor
from gpiozero import Robot
from time import sleep
import RPi.GPIO as GPIO


robot = Robot(left = (27, 17), right = (22, 23))
sensor = DistanceSensor(echo=8, trigger=24)

while True:
  distance_to_object = sensor.distance * 100
  print(distance_to_object)
  if distance_to_object  <= 25:
    robot.backward()
    sleep(1)
    robot.left()
    sleep(1)
  else:
    robot.forward()
    sleep(0.1)



