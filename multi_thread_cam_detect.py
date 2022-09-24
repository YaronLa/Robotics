# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 17:33:14 2022

@author: flemm
"""

import cv2
from time import sleep, perf_counter
from threading import Thread
import time
import numpy as np

#Pipline
def gstreamer_pipeline(capture_width=1024, capture_height=720, framerate=30):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc !"
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (
            capture_width,
            capture_height,
            framerate,
        )
    )


class CamObject:
    def __init__(self, stream_id=0):
        #self.cam      = cv2.VideoCapture(stream_id, cv2.CAP_DSHOW)
        self.cam      = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)        
        self.grabbed , self.frame = self.cam.read()
        self.running = True
        self.task     = Thread(target=self.update, args=()).start()
        
    def update(self):
       while self.running:
           self.grabbed , self.frame = self.cam.read()
       self.cam.release()
       
    def read(self):
        return self.grabbed, self.frame
    
    def stop_task(self):
        self.running = False


def detector(corners, markerLength, camera_matrix, dist_coeffs):
    #arlo.stop()
    #sleep(1)
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
    dist = np.linalg.norm(rvec-tvec)
    tvec = tvec.reshape((3,))
    dist = np.linalg.norm(tvec)
    theta = np.arccos(np.dot((tvec/dist),ez))
    signfunc = np.sign(np.dot(tvec,ex))
    ang_deg = signfunc * np.rad2deg(theta)
    
    return ang_deg, signfunc

#Info used
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
camera_matrix = np.array([ 1651, 0.0,512, 0, 1651, 360, 0, 0, 1]).reshape(3,3)
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape(5,1)
markerLength = 150
ex = ([1,0,0])
ez = ([0,0,1])


#starting our cam
cam = CamObject() # Cam running in other thread than proces
sleep(0.5) # Some cams need to warm up with a few frames. Apparently?
#cam = cv2.VideoCapture(0, cv2.CAP_DSHOW) #Cam running in same thread as proces
#cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)

# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

#counter = 0 #just to test optimized
#start_time = perf_counter()
while cv2.waitKey(4) == -1: # Wait for a key pressed event
    retval , frameReference = cam.read() # Read frame
    
    #Object detection
    corners, ids, rejected = cv2.aruco.detectMarkers(frameReference, dict)
    cv2.aruco.drawDetectedMarkers(frameReference,corners)
    cv2.imshow(WIN_RF, frameReference)
    
    #If object detected
    if corners:
        ang_deg, signfunc = detector(corners, markerLength, camera_matrix, dist_coeffs)
    
    
    """
    #Still just for opti
    counter += 1
    temp_time = perf_counter()
    if temp_time-start_time > 10:
        break
    """



end_time = perf_counter()
print(f'It took {end_time- start_time: 0.2f} second(s) to complete.')
print(counter)

cam.stop_task()
cv2.destroyAllWindows()


















