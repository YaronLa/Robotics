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


leftSpeed, rightSpeed, time = 70, 70    #Speed between [40;127],
time                        = 5         #time is in seconds





"""Helping to find time it takes to turn 90 degrees"""
def turn_ninety(leftSpeed, rightSpeed, turn_direc = "left"): #it will spin clockwise if turn_direc is set to anything else than left. 
    spin_lw, spin_rw = 1 , 0    #Choose spin direction
    if turn_direc == "left":
        spin_lw, spin_rw = 0 , 1
        
    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, spin_lw, spin_rw))
    while True:
        if (time.perf_counter() - start_time > 5.0):
            print(arlo.stop())
            break



    
    


turn_ninety(leftSpeed, rightSpeed, turn_direc = "left")  

    
print(arlo.go_diff(0, 0, 1, 1))  
