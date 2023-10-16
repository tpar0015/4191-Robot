import time
from MotorControl.drive_new import Drive
from multiprocessing import Manager
from processing import MultiProcess
from pins import *
from electromagnet import Electromagnet




class Control:
    def __init__(self):
        self.manager = Manager()
        processes = MultiProcess(self.manager, ["front", "left", "right"])
        drive_control = Drive([0,0,0])

    def start(self):
        pass


if __name__ == "__main__":
    with Manager() as manager:
        processes = MultiProcess(manager, ["drawback"])
        processes.start_processes()
        while True:
            distance = processes.get_ultrasonic("drawback")
            print("Distance: %.2f mm" % (distance))
            time.sleep(0.2)