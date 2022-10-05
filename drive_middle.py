import numpy as np
import cv2
import actions
import robot
import slow-selflocalize

arlo = robot.Robot()

def driving_strat(middle, pose):
    delta_y = middle[1]-pose[1]
    delta_x = middle[0]-pose[0]
    dist = np.sqrt(delta_x**2 + delta_y**2)
    theta = np.arccos(delta_y/dist) 
    theta_new = (pose[3]-theta)*180/np.pi
    return theta_new, dist

def drive_to_middle(theta, dist):
    sign, theta = np.sign(theta), np.abs(theta)
    actions.turn(theta, sign)
    actions.forward_mm(dist)