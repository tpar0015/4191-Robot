# Provides functions to stop motors depending on rotary encoder output
from gpiozero import RotaryEncoder


pin_rotary_a = 1    #? change later
pin_rotary_b = 2

rotor = RotaryEncoder(pin_rotary_a, pin_rotary_b, max_steps=0)

last_position = None
def printPosition():
    position = rotor.steps
    if last_position == None or position != last_position:
        print(position)
    last_position = position

# #! review upon testing
# def stopAfterTicks(ticks, motor1="", motor2=""):
#     initialPosition = enc.position
#     diff = 0
#     while abs(diff) < ticks:
#         newPosition = enc.position
#         diff = newPosition - initialPosition
#     if motor1:
#         motor1.off()
#     if motor2:
#         motor2.off()