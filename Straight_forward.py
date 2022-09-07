# -*- coding: utf-8 -*-
"""
Created on Mon Sep  5 18:33:42 2022

@author: flemm
"""

import robot
import time
from time import sleep


arlo = robot.Robot()
sleep(1)


leftSpeed, rightSpeed = 100, 100    #Speed between [40;127],
t                        = 8         #time is in seconds


"""Helping to find time it takes to drive one meter"""
def one_meter(leftSpeed, rightSpeed, t): #Getting it to run straight by adjusting l,r-speed and time
    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    while True:
        if (int(time.perf_counter() - int(start_time)) > t):
            print(arlo.stop())
            break





    
    


one_meter(leftSpeed, rightSpeed, time)


    
    
print(arlo.go_diff(0, 0, 1, 1))  
    
    

