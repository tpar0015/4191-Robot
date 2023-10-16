import time
from MotorControl.drive_new import Drive
from MotorControl.motorctl_new import Motor
from multiprocessing import Manager
from processing import MultiProcess
from pins import *
from electromagnet import Electromagnet




class Control:
    def __init__(self):
        self.manager = Manager()
        self.processes = MultiProcess(self.manager, ["drawback"])
        self.drive_control = Drive([0,0,0])
        self.electromagnet = Electromagnet(gpio_pin=PINS["electromagnet"])

        self.drawback_motor = Motor(PINS["motor3_en"], PINS["motor3_a"], PINS["motor3_b"])
        self.drawback_distance = 50
        self.package_holder_home = 250
        self.ultrasonic_error = 20

    def start(self):
        
        # Stage 1: Camera Scan

        ##############################
        # Stage 2: Drive towards bin

        ##############################
        # Stage 3: Drawback & Fire

        # Check where the magnet is
        current_dist = self.processes.get_ultrasonic("drawback")
        if self.package_holder_home - self.ultrasonic_error < current_dist < self.package_holder_home + self.ultrasonic_error:
            # Magnet is at home position
            while current_dist > self.drawback_distance:
                
                current_dist = self.processes.get_ultrasonic("drawback")
        ##############################
        # Stage 4: Drive backwards 



if __name__ == "__main__":
    with Manager() as manager:
        processes = MultiProcess(manager, ["drawback"])
        processes.start_processes()
        while True:
            distance = processes.get_ultrasonic("drawback")
            print("Distance: %.2f mm" % (distance))
            time.sleep(0.2)