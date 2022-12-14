# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 11:19:30 2022

@author: flemm
"""

import robot
import time
from time import sleep

print(type(int(time.perf_counter())))



arlo = robot.Robot()
sleep(1)


driveleftSpeed, driverightSpeed= 70, 69    #Speed between [40;127],
turnleftSpeed, turnrightSpeed = 70, 70
def one_meter(leftSpeed, rightSpeed): #Getting it to run straight by adjusting l,r-speed and>
    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    while True:
        if (time.perf_counter() - start_time > 2.235):
            print(arlo.stop())
            break

def turn_ninety(leftSpeed, rightSpeed, turn_direc = "left"): #it will spin clockwise if turn>
    spin_lw, spin_rw = 1 , 0    #Choose spin direction
    if turn_direc == "left":
        spin_lw, spin_rw = 0 , 1

    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, spin_lw, spin_rw))
    while True:
        if (time.perf_counter() - start_time > 0.675):
            print(arlo.stop())
            break



def drive_in_square(driveleftSpeed, driverightSpeed,
                    turnleftSpeed, turnrightSpeed,
                    n_times = 12):

    buffer_time = 0.5
    for i in range(n_times):
        one_meter(driveleftSpeed, driverightSpeed)
        sleep(2.35+buffer_time)
        turn_ninety(turnleftSpeed, turnrightSpeed, turn_direc = "left")
        sleep(0.67+buffer_time)




#one_meter(leftSpeed, rightSpeed, time)

#turn_ninety(leftSpeed, rightSpeed, time, turn_direc = "left")

drive_in_square(driveleftSpeed, driverightSpeed, turnleftSpeed, turnrightSpeed)

    
    
print(arlo.go_diff(0, 0, 1, 1)) 

