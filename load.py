from electromagnet import Electromagnet
from MotorControl.motorctl_new import Motor


def load(magnet, motor):
    # Advance magnet
    motor.forward()
    sleep(3)    #? need to check how long the motor needs to travel
    motor.stop()

    # Turn on
    magnet.turn_on()

    # Reverse magnet
    motor.backward()
    sleep(3)
    motor.stop

    
if __name__ == '__main__':
    magnet = Electromagnet()
    motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])

    load(magnet, motor)