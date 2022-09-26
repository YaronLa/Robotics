

import cv2 # Import the OpenCV library
import cv2.aruco

import numpy as np

print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
cam = cv2.VideoCapture(0)

if not cam.isOpened(): # Error
    print("Could not open camera")
    exit(-1)

# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
ez = np.array([0,0,1])
ex = np.array([1,0,0])
camera_matrix = np.array([ 1651, 0.0,512, 0, 1651, 360, 0, 0, 1]).reshape(3,3)
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape(5,1)
markerLength = 150


while cv2.waitKey(4) == -1: # Wait for a key pressed event
    retval, frameReference = cam.read() # Read frame
    
    if not retval: # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frameReference, arucoDict,
        parameters=arucoParams)

    cv2.aruco.drawDetectedMarkers(frameReference,corners)
    print(ids)
    cv2.imshow("billede",frameReference)
    if corners: 
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        #print(rvec, tvec)
        #np.dot(tvec,ex)*
        tvec = tvec
        dist = np.linalg.norm(tvec)
        print(dist)
        if 950 < dist < 1050: 
            print('TAKING PIC IN 3')
            sleep(1)
            print('TAKING PIC IN 2')
            sleep(1)
            print('TAKING PIC IN 1')
            sleep(1)
            print('"click"')
            return_value, image = cam.read()
            cv2.imwrite(str(dist)+str(corners)+'.png', image)
            sleep(2)
            print('READY')
        if 1950 < dist < 2050: 
            print('TAKING PIC IN 3')
            sleep(1)
            print('TAKING PIC IN 2')
            sleep(1)
            print('TAKING PIC IN 1')
            sleep(1)
            print('"click"')
            return_value, image = cam.read()
            cv2.imwrite(str(dist)+str(corners)+'.png', image)
            sleep(2)
            print('READY')
# Finished successfully




