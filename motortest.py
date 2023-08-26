from time import sleep
from motorctl import motor_x

while True:
    motor_x(1)

    sleep(2)

    motor_x(-1)

    sleep(2)