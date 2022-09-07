import robot
import time
from time import sleep


arlo = robot.Robot()
sleep(1)


leftSpeed, rightSpeed = 70, 69    #Speed between [40;127],
t                        = 8         #time is in seconds


"""Helping to find time it takes to drive one meter"""
def one_meter(leftSpeed, rightSpeed, t): #Getting it to run straight by adjusting l,r-speed >
    start_time = time.perf_counter()
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    while True:
        if ((float(time.perf_counter()) - float(start_time)) > 2.235*2.0):
            print(arlo.stop())
            break









one_meter(leftSpeed, rightSpeed, time)




print(arlo.go_diff(0, 0, 1, 1))






    
    


one_meter(leftSpeed, rightSpeed, time)


    
    
print(arlo.go_diff(0, 0, 1, 1))  
    
    

