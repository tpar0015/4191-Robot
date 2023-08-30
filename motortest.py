import rotary
import motorctl

Robot = motorctl.Motor()


# rotary.printPosition()
while True:
    Robot.forward(1,1)