# Provides functions to stop motors depending on rotary encoder output
import rotaryio

pin_rotary_a = 1    #? change later
pin_rotary_b = 2

enc = rotaryio.IncrementalEncoder(pin_a=pin_rotary_a, pin_b=pin_rotary_b, divisor=1)

last_position = None
def printPosition():
    position = enc.position
    if last_position == None or position != last_position:
        print(position)
    last_position = position

#! review upon testing
def stopAfterTicks(ticks, motorlist):
    initialPosition = enc.position
    diff = 0
    while abs(diff) < ticks:
        newPosition = enc.position
        diff = newPosition - initialPosition
    for motor in motorlist:
        motor.off()