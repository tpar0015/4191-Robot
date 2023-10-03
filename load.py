from electromagnet import Electromagnet
from MotorControl.motorctl_new import Motor
from time import sleep
from pins import *

draw_times = {
    "A":    2,
    "B":    3,
    "C":    4
}

def load(magnet, motor):
    # Draw back launcher
    motor.forward()
    
    sleep(draw_times["A"])
    
    # Stop launcher
    motor.stop()

    # Turn on
    magnet.turn_on()

    # Reverse magnet
    motor.backward()
    sleep(draw_times["A"])
    motor.stop

    
if __name__ == '__main__':
    magnet = Electromagnet()
    motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])

    load(magnet, motor)

    sleep(5)
    motor.turn_off()