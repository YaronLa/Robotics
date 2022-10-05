import numpy as np
import cv2
import actions
import robot
import Exercise3_findthelandmark as cam_imp
import particles
import Self_localization_slow as sls

arlo = robot.Robot()
camera = cam_imp.CamOpject()
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

def driving_strat(middle, pose):
    delta_y = middle[1]-pose[1]
    delta_x = middle[0]-pose[0]
    dist = np.sqrt(delta_x**2 + delta_y**2)
    theta = np.arccos(delta_y/dist) 
    theta_new = (pose[2]-theta)*180/np.pi
    return theta_new, dist

def drive_to_middle():
    pose = []
    id_lst = []
    parties_lst = []
    while len(id_lst) < 2:
        retval , frameReference = cam.read() # Read frame
        corners, ids, rejected = cv2.aruco.detectMarkers(frameReference, dict)
        cv2.aruco.drawDetectedMarkers(frameReference,corners)
        if not corners:
            actions.scan_for_object(camera, dict)
            sleep(1)
        if corners:
            if len(id_lst) == 0:

                theta, x, y, parties = sls.self_locate()
                parties_lst.append(parties)
            elif len(id_lst) == 1:
                theta, x, y, parties = sls.self_locate(parties_lst[0])
                pose.append(x)
                pose.append(y)
                pose.append(theta)
            if ids in id_lst:
                continue
            else:
                id_lst.append(ids)
    return pose
            #arlo.stop()
           # dist, ang_deg, signfunc = actions.detector(corners, markerLength, camera_matrix, dist_coeffs)
            #actions.drive_to_object(dist, ang_deg, signfunc)

pose = drive_to_middle()
theta_new, dist = driving_strat([150,0], pose)
sign, theta = np.sign(theta_new), np.abs(theta_new)
actions.turn(theta, sign)
actions.forward_mm(dist)
arlo.stop()
