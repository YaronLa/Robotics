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


leftSpeed, rightSpeed = 60, 60    #Speed between [40;127],



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

#turn_ninety(leftSpeed, rightSpeed)




def turn_degrees(degrees, sign, leftSpeed = leftSpeed , rightSpeed= rightSpeed, turn_direc = "left"): #it will spin clockwise if turn>
    scalar = degrees/ 90
    
    spin_lw, spin_rw = 1 , 0    #Choose spin direction
    if sign == -1:
        spin_lw, spin_rw = 0 , 1

    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, spin_lw, spin_rw))
    while True:
        if (time.perf_counter() - start_time > 0.675*scalar):
            print(arlo.stop())
            break

#turn_degrees(180)



