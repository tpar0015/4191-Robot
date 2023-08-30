import MotorControl.rotary as rotary
import motorctl

Robot = motorctl.Motor()

while True:
    # rotary.printPosition()
    Robot.forward(1,1)