# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 14:56:56 2022

@author: flemm
"""


import robot
import time
from time import sleep


arlo = robot.Robot()
sleep(1)


leftSpeed, rightSpeed = 60, 85    #Speed between [40;127],



"""Helping to find time it takes to drive one meter"""
def one_meter(leftSpeed, rightSpeed): #Getting it to run straight by adjusting l,r-speed >
    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    
    state_time = time.perf_counter()
    state = 1
    while True:
        if ((float(time.perf_counter()) - float(start_time)) > 10.0):
            print(arlo.stop())
            break
        
        
        if ((float(time.perf_counter()) - float(state_time)) > 2.0) and state == 1:
            print(arlo.go_diff(rightSpeed, leftSpeed, 1, 1))
            state = 0
            state_time = time.perf_counter()
            
        if ((float(time.perf_counter()) - float(state_time)) > 2.0) and state == 1:
            print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
            state = 1
            state_time = time.perf_counter()
            
            







one_meter(leftSpeed, rightSpeed)


    
    
print(arlo.go_diff(0, 0, 1, 1))  