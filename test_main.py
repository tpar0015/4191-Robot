import time
from MotorControl.drive_new import Drive
from MotorControl.motorctl_new import Motor
from multiprocessing import Manager
from processing import MultiProcess
from pins import *
from electromagnet import Electromagnet
from camera_pyFile.QRdetect import readQR
import numpy as np


class Control:
    def __init__(self):
        self.manager = Manager()
        self.position = {           
            "x": 0,             # Positive x is from loading zone to bins
            "y": 0,             # Positive y is left to right from loading zone side
            "angle": 0          # Positive angle is anticlockwise from the right (radians in (-pi,pi])
        }
        self.processes = MultiProcess(self.manager, ["drawback"])
        self.drive_control = Drive([0,0,0])
        self.electromagnet = Electromagnet(gpio_pin=PINS["electromagnet"])

        self.drawback_motor = Motor(-1, PINS["motor3_a"], PINS["motor3_b"])
        self.drawback_distance = 50         # drawback to fire
        self.ready_distance = 60            # ready position ~70% of fire drawback
        self.package_holder_home = 250      # home position after firing
        self.ultrasonic_error = 20          # uncertainty in ultrasonic sensor distance reading

    def get_bin_position(self, cam_result):
        match cam_result:
            case 1:  # Bin A
                return {"x": 130, "y": 0}
            case 2:  # Bin B
                return {"x": 130, "y": 60}
            case 3:  # Bin C
                return {"x": 130, "y": 120}
            case _:
                return None
        

    def start(self):
        # Stage 0: Home and drawback 70%

        # Check where the magnet is
        current_dist = self.processes.get_ultrasonic("drawback")

        # Draw forward to home position
        print("Drawing to home position")
        if current_dist < self.package_holder_home - self.ultrasonic_error:
            # Move to package holder
            while current_dist < self.package_holder_home - self.ultrasonic_error:
                #self.drawback_motor.set_speed(100)
                self.drawback_motor.forward()
                current_dist = self.processes.get_ultrasonic("drawback")
            self.drawback_motor.stop()
            self.electromagnet.turn_on()

        # Drawback to ready position
        print("Drawing to ready position")
        if current_dist > self.ready_distance + self.ultrasonic_error:
            # Move to ready position
            while current_dist > self.ready_distance + self.ultrasonic_error:
                #self.drawback_motor.set_speed(100)
                self.drawback_motor.backward()
                current_dist = self.processes.get_ultrasonic("drawback")
            self.drawback_motor.stop()

        ##############################

        # Stage 1: Read QR code at camera
        cam_result = None

        while cam_result is None:
            cam_result = readQR()

        print("Read QR code with result", cam_result)

        ##############################
        # Stage 2: Orient towards bin
        bin_position = self.get_bin_position(cam_result)

        # Calculate required turn angle
        x_deviation = bin_position["x"] - self.position["x"]
        y_deviation = bin_position["y"] - self.position["y"]
        target_angle = np.arctan2(y_deviation, x_deviation)

        #? TODO: Turn to target angle

        ##############################
        # Stage 3: Drawback & Fire

        # Check where the magnet is
        current_dist = self.processes.get_ultrasonic("drawback")

        if self.ready_distance - self.ultrasonic_error < current_dist < self.ready_distance + self.ultrasonic_error:
            # Magnet is at ready position
            while current_dist > self.drawback_distance:
                #self.drawback_motor.set_speed(100)
                self.drawback_motor.backward()
                current_dist = self.processes.get_ultrasonic("drawback")
            self.drawback_motor.stop()
            # Fire
            self.electromagnet.turn_off()
        ##############################
        # Stage 4: Orient to home position 
        
        #? TODO: make this work
        target_angle = 0
        self.drive_control.drive_forward(-1)


if __name__ == "__main__":
    with Manager() as manager:
        processes = MultiProcess(manager, ["drawback"])
        processes.start_processes()

        robot = Control()

        while True:
            # distance = processes.get_ultrasonic("drawback")
            # print("Distance: %.2f mm" % (distance))
            # time.sleep(0.2)

            robot.start()