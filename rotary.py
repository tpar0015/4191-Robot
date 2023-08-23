# Provides functions to stop motors depending on rotary encoder output
from gpiozero import RotaryEncoder


pin_rotary_a = 20    #? change later
pin_rotary_b = 21

rotor = RotaryEncoder(pin_rotary_a, pin_rotary_b, max_steps=100)

def printPosition():
    last_position = None
    while True:
        position = rotor.value
        # if last_position == None or position != last_position:
        print(position)
        # last_position = position

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