# -*- coding: utf-8 -*-
"""
Created on Thu Sep  8 11:04:02 2022

@author: flemm
"""

import math
import robot
import time
from time import sleep
import random

arlo = robot.Robot()
sleep(1)


dist = []
for i in range(5):
    val = arlo.read_front_ping_sensor()
    dist.append(val)
    sleep(1)
    
print(dist)


