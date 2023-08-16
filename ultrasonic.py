
import PRi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

# Assigning trig_pin and echo_pin to GPIO pins 11 and 12
TRIG_PIN = 11
ECHO_PIN = 12

# Set GPIO distrection
GPIO.setup(TRIG_PIN, GPIO.OUT) # Set trig as output
GPIO.setup(ECHO_PIN, GPIO.IN)  # Set echo as input
GPIO.OUTPUT(TRIG_PIN, GPIO.LOW) # Drive trig to 0V

time.sleep(2)
GPIO.output(TRIG_PIN, GPIO.HIGH) # Set trig to high

time.sleep(0.00001)

GPIO.output(TRIG_PIN, GPIO.LOW)

while GPIO.input(ECHO_PIN) == 0:
    pulse_send = time.time()
while GPIO.input(ECHO_PIN) == 1:
    pulse_received = time.time()

pulse_duration = round((pulse_received - pulse_send)/2.2)
dist = 34000*pulse_duration

# GPIO.cleanup() # resets all ports/pins

