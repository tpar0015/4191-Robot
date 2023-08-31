# Provides functions to stop motors depending on rotary encoder output
import time
import RPi.GPIO as GPIO
pin_rotary_a = 20    #? change later
pin_rotary_b = 21



class RotaryEncoder:
    """Rotary Encoder class to determine linear and angular velocities for feedback into control loop"""
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.prev_count = 0
        self.state = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)

        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self.encoder_callback)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self.encoder_callback)

        self.time_check = time.time()

    def encoder_callback(self, pin):
        a_state = GPIO.input(self.pin_a)
        b_state = GPIO.input(self.pin_b)

        if self.state is not None:
            if (self.state[0], self.state[1]) == (0,1):
                if (a_state, b_state) == (1,1):
                    self.count += 1
                elif (a_state, b_state) == (0,0):
                    self.count -= 1
            elif (self.state[0], self.state[1]) == (1,0):
                if (a_state, b_state) == (0,1):
                    self.count += 1
                elif (a_state, b_state) == (1,0):
                    self.count -= 1

        self.state = (a_state, b_state)

    def wheel_velocity(self, radius):
        """Returns wheel velocity"""
        current_time = time.time()
        time_diff = current_time - self.time_check

        # Calculate Count Change
        count_change = self.count - self.prev_count
        # Angular Velocity
        angular_velocity = count_change / time_diff

    def get_count(self):
        """Returns count"""
        return self.count
    
    def reset_count(self):
        """Resets counter"""
        self.count = 0

    def cleanup(self):
        """Cleans up GPIO pins"""
        GPIO.remove_event_detect(self.pin_a)
        GPIO.remove_event_detect(self.pin_b)
        GPIO.cleanup()


            
