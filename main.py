## Main running loop for the package delivery robot
## Implements a state machine with two states: 'Load' and 'Fire'


from electromagnet import Electromagnet
from MotorControl.motorctl_new import Motor
from pins import *
from load import load

mag = Electromagnet()
launcher = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)


# Load cycle
load(mag, launcher)


# Fire cycle
