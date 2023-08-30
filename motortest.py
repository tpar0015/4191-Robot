# import rotary
import RPi.GPIO as GPIO
import motorctl

Robot = motorctl.Motor()

if __name__ == 'main':    # rotary.printPosition()
    Robot.forward(100,100)
    sleep(5)
    GPIO.cleanup()

