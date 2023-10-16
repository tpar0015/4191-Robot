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

        self.drawback_motor = Motor(-1, PINS["motor3_a"], PINS["motor3_b"])
        self.drawback_distance = 50
        self.package_holder_home = 250
        self.ultrasonic_error = 20

    def start(self):
        
        # Stage 1: Camera Scan
        cam_result = None

        ##############################
        # Stage 2: Drive towards bin
        bin_position = self.get_bin_position(cam_result)


        ##############################
        # Stage 3: Drawback & Fire

        # Check where the magnet is
        current_dist = self.processes.get_ultrasonic("drawback")
        # Magnet is too far away from package holder
        if current_dist > self.package_holder_home + self.ultrasonic_error:
            # Move to package holder
            self.electromagnet.turn_on()
            while current_dist > self.package_holder_home + self.ultrasonic_error:
                #self.drawback_motor.set_speed(100)
                self.drawback_motor.forward()
                current_dist = self.processes.get_ultrasonic("drawback")
            self.drawback_motor.stop()

        if self.package_holder_home - self.ultrasonic_error < current_dist < self.package_holder_home + self.ultrasonic_error:
            # Magnet is at home position
            while current_dist > self.drawback_distance:
                #self.drawback_motor.set_speed(100)
                self.drawback_motor.backward()
                current_dist = self.processes.get_ultrasonic("drawback")
            self.drawback_motor.stop()
            # Fire
            self.electromagnet.turn_off()
        ##############################
        # Stage 4: Drive backwards 
        self.drive_control.drive_forward(-1)


if __name__ == "__main__":
    with Manager() as manager:
        processes = MultiProcess(manager, ["drawback"])
        processes.start_processes()
        while True:
            distance = processes.get_ultrasonic("drawback")
            print("Distance: %.2f mm" % (distance))
            time.sleep(0.2)