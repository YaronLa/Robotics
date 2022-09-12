# -*- coding: utf-8 -*-
"""
Created on Mon Sep 12 10:30:22 2022

@author: flemm
"""

import math
import robot
import time
from time import sleep
import random


arlo = robot.Robot()
sleep(1)


leftSpeed, rightSpeed = 60, 60    #Speed between [40;127],
t = 30.0
safety_dist = 200


def NOT(a):
    if a != 0 and a != 1:
        return "error"
    binary = 0
    if not a == True:
        binary = 1
    return binary


def scan(safety_dist = 200):
    turn_direc = 1 #random.randint(0, 1)
    while True:
        arlo.go_diff(40, 40, turn_direc, NOT(turn_direc))
        if (arlo.read_front_ping_sensor() >= safety_dist+5 
            and arlo.read_left_ping_sensor() >= safety_dist+5 
            and arlo.read_right_ping_sensor() >= safety_dist+5):
            print(arlo.stop())
            break    


def self_drive(leftSpeed, rightSpeed, t, safety_dist = 200): #Getting it to run straight by adjusting l,r-speed >
    start_time = time.perf_counter()

    arlo.go_diff(leftSpeed, rightSpeed, 1, 1)
    while True:        
        if ((float(time.perf_counter()) - float(start_time)) > t):
            print(arlo.stop())
            break
        
        if (arlo.read_front_ping_sensor() <= safety_dist 
            or arlo.read_left_ping_sensor() <= safety_dist
            or arlo.read_right_ping_sensor() <= safety_dist):
              
            arlo.stop()
            sleep(0.5)
            scan()
            arlo.go_diff(leftSpeed, rightSpeed, 1, 1)
                
                
        
                
self_drive(leftSpeed, rightSpeed, t)

print(arlo.stop())
    
