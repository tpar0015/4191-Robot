# Purpose: To create a class that will handle all of the multiprocessing
import time
from pins import *
from multiprocessing import Queue, Process
from ultrasonic import Ultrasonic
from MotorControl.rotary_new import RotaryEncoder



class MultiProcess:
    def __init__(self, ultrasonic_names):
        self.ultrasonic_names = ultrasonic_names
        self.ultrasonic_dict = {}
        self.ultrasonic_queue = Queue(maxsize=10)
        self.ultrasonics = []
        self.initialize_ultrasonics(ultrasonic_names)
        self.rotary_queue = Queue()


    def initialize_ultrasonics(self, names):
        """Initializes the ultrasonic sensors and adds them to the dictionary"""
        for name in names:
            self.ultrasonic_dict[name] = 0

    def read_ultrasonic_queue(self):
        """Reads the ultrasonic queue and updates the ultrasonic dictionary"""
        readouts  = self.ultrasonic_queue.get()
        for i in range(len(self.ultrasonic_names)):
            self.ultrasonic_dict[self.ultrasonic_names[i]] = readouts[i]

    def ultrasonic_loop(self):
        """Runs the ultrasonic sensors and updates the ultrasonic dictionary"""
        for i in range(len(self.ultrasonic_names)):
            self.ultrasonics.append(Ultrasonic(PINS[f"sonar_trig{i}"], PINS[f"sonar_echo{i}"]))          # Need pins
        while True:
            readouts = []
            for ultrasonic in self.ultrasonics:
                dist = ultrasonic.single_iteration()
                readouts.append(dist)
            self.ultrasonic_queue.put(readouts)
            
            time.sleep(0.1)

    def get_ultrasonic(self, name):
        """Returns the ultrasonic dictionary"""
        return self.ultrasonic_dict[name]
    

    def rotary_loop(self):
        """Runs the rotary encoders and updates the rotary queue"""
        left_encoder = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"])
        right_encoder = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"])
        while True:
            self.rotary_queue.put((left_encoder.count, right_encoder.count))
            time.sleep(0.1) 

    def get_rotary(self):
        """Returns the rotary queue"""
        return self.rotary_queue.get()

    def start_processes(self):
        """Initializes the processes"""
        self.ultrasonic_process = Process(target=self.ultrasonic_loop)
        self.rotary_process = Process(target=self.rotary_loop)
        self.ultrasonic_process.start()
        self.rotary_process.start()

    def terminate_processes(self):
        """Terminates the processes"""
        self.ultrasonic_process.terminate()
        self.rotary_process.terminate()
        self.ultrasonic_process.join()
        self.rotary_process.join()
