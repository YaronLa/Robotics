# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 11:19:32 2022
@author: flemm
"""

import robot
import time
from time import sleep


arlo = robot.Robot()
sleep(1)


leftSpeed, rightSpeed = 70, 70    #Speed between [40;127],




def calibrate(leftSpeed, rightSpeed, turn_direc = "left"))
    spin_lw, spin_rw = 1 , 0    #Choose spin direction
    if turn_direc == "left":
        spin_lw, spin_rw = 0 , 1
        
    start_time = time.perf_counter()
    start_point = arlo.read_front_ping_sensor()
    print(arlo.go_diff(leftSpeed, rightSpeed, spin_lw, spin_rw))
    print(start_point)
    while True:
        print(arlo.read_front_ping_sensor())
        print(arlo.go_diff(leftSpeed, rightSpeed, spin_lw, spin_rw))
        if read_front_ping_sensor() == start_point
            print(arlo.stop())
            print(time.perf_counter() - start_time)
            break

    
    


calibrate(leftSpeed, rightSpeed, turn_direc = "left")  

    
print(arlo.go_diff(0, 0, 1, 1))  