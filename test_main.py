from MotorControl.drive_new import Drive
from processing import MultiProcess
from pins import *
from electromagnet import Electromagnet




class Control:
    def __init__(self):
    
    processes = MultiProcess(["front", "left", "right"])
    drive_control = Drive([0,0,0])

    def start(self):
        pass


if __name__ == "__main__":

    pass
