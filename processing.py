# Purpose: To create a class that will handle all of the multiprocessing
import time
from pins import *
from multiprocessing import Queue, Process, Manager
from ultrasonic import Ultrasonic
from MotorControl.rotary_new import RotaryEncoder



class MultiProcess:
    def __init__(self, manager, ultrasonic_names):
        self.manager = manager
        self.ultrasonic_names = ultrasonic_names
        self.ultrasonic_dict = self.manager.dict()
        self.ultrasonics = []
        self.initialize_ultrasonics(ultrasonic_names)

        self.left_count = self.manager.Value("i", 0)
        self.right_count = self.manager.Value("i", 0)


    def initialize_ultrasonics(self, names):
        """Initializes the ultrasonic sensors and adds them to the dictionary"""
        for name in names:
            self.ultrasonic_dict[name] = 0

    
    def ultrasonic_loop(self):
        """Runs the ultrasonic sensors and updates the ultrasonic dictionary"""
        for i in range(len(self.ultrasonic_names)):
            self.ultrasonics.append(Ultrasonic(PINS[f"sonar_trig{i}"], PINS[f"sonar_echo{i}"]))          # Need pins
        while True:
            readouts = []
            for ultrasonic, name in zip(self.ultrasonics, self.ultrasonic_names):
                dist = ultrasonic.single_iteration()
                self.ultrasonic_dict[name] = dist
            time.sleep(0.1)

    def get_ultrasonic(self, name):
        """Returns the ultrasonic dictionary"""
        # print("Ultrasonic_Queue: ", self.ultrasonic_queue.get())
        return self.ultrasonic_dict[name]
    

    def rotary_loop(self):
        """Runs the rotary encoders and updates the rotary queue"""
        left_encoder = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"])
        right_encoder = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"])
        while True:
            self.left_count.value = left_encoder.get_count()
            self.right_count.value = right_encoder.get_count()


    def get_rotary(self):
        """Returns the rotary queue"""
        return self.left_count.value, self.right_count.value

    def start_processes(self, ultrasonic=True, rotary=True):
        """Initializes the processes"""
        if ultrasonic:
            self.ultrasonic_process = Process(target=self.ultrasonic_loop)
        if rotary:
            self.rotary_process = Process(target=self.rotary_loop)
            self.rotary_process.start()
        self.ultrasonic_process.start()

    def reset_count(self):
        self.left_count.value = 0
        self.right_count.value = 0

    def terminate_processes(self):
        """Terminates the processes"""
        self.ultrasonic_process.terminate()
        self.rotary_process.terminate()
        self.ultrasonic_process.join()
        self.rotary_process.join()
