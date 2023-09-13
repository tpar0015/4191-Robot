"""Calculates distance per tick"""
import math
import time
from drive_new import Drive
import RPi._GPIO as GPIO



class Calibrate:
    def __init__(self):
        self.Controller = Drive([0,0,0], [0,0,0], [0,0,0])
        self.dist_per_tick = (50 * 2 * math.pi) / (74.83 * 48)  # Distance per tick in mm
        self.wheel_radius = 50
    def drive_one_meter(self, speed):          
        self.Controller.set_speed(speed)
        self.Controller.drive_forward(100, self.Controller.pid_forward) 
    def get_num_ticks(self):
        return self.Controller.left_encoder.count + self.Controller.right_encoder.count
    
    def loop(self):
        # speeds = list(range(50, 110,10))
        speeds = [50, 75, 100]
        results = []
        i = 0
        while i < len(speeds):
            # Drive Time
            time_to_drive = float(input("Driving Time (s): "))
            dist_to_drive = time_to_drive * self.dist_per_tick

            start_time = time.time()
            speed = speeds[i]
            print("Driving 1m at speed: ", speed)
            # Ticks / Second
            drive_time = time.time() - start_time
            ticks = self.get_num_ticks()
            # 3591.84 == Counts per revolution, * 2 encoders
            num_revolutions = ticks / (2*3591.84)
            dist = num_revolutions * 2*math.pi * self.wheel_radius # mm
            velocity = dist / drive_time # mm / s

            bool = input("Did robot drive 1m? (y/n)")
            if bool == "Y" or bool == "y":
                results.append((velocity, self.get_num_ticks()))
                i += 1
            else:
               pass 

        return results
    
    def get_distance_per_tick(self):
        output = self.loop()
        distance_per_tick = 0
        for _, ticks in output:
            distance_per_tick += 1000 / ticks

        return distance_per_tick / len(output)

    def calibrate_stopping_ticks(self):
        num_ticks = 1000
        x = 0
        while self.Controller.left_encoder.count + self.Controller.right_encoder.count < num_ticks:
            print(self.Controller.left_encoder.count + self.Controller.right_encoder.count)
            self.Controller.right_motor.set_speed(100)
            self.Controller.left_motor.set_speed(100)
            x = self.Controller.left_encoder.count + self.Controller.right_encoder.count
        self.Controller.left_motor.stop()
        x1 = self.Controller.left_encoder.count
        self.Controller.right_motor.stop()
        x2 = self.Controller.right_encoder.count
        x = x1 + x2
        print("Stopped")
        time.sleep(1)
        y = self.Controller.left_encoder.count + self.Controller.right_encoder.count
        return y - x
if __name__=="__main__":
    calibration = Calibrate()
    print(calibration.calibrate_stopping_ticks())
    GPIO.cleanup()
                
