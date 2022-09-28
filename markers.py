# This script shows how to open a camera in OpenCV and grab frames and show these.
# Kim S. Pedersen, 2022

import cv2 # Import the OpenCV library
import cv2.aruco
from time import sleep
import robot
import numpy as np

arlo = robot.Robot()
ez = np.array([0,0,1])
ex = np.array([1,0,0])

print("OpenCV version = " + cv2.__version__)
def gstreamer_pipeline(capture_width=1280, capture_height=720, framerate=30):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc !"
        "videobox autocrop=true !"
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (
            capture_width,
            capture_height,
            framerate,
        )
    )


print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)
if not cam.isOpened(): # Error
    print("Could not open camera")
    exit(-1)

# Open a window
#WIN_RF = "Example 1"
#cv2.namedWindow(WIN_RF)
#cv2.moveWindow(WIN_RF, 100, 100)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
#(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
#	parameters=arucoParams)
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

camera_matrix = np.array([ 1651, 0.0,512, 0, 1651, 360, 0, 0, 1]).reshape(3,3)
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape(5,1)
markerLength = 150

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    retval, frameReference = cam.read() # Read frame
    
    if not retval: # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    # Show frames
    #cv2.imshow(WIN_RF, frameReference)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frameReference, arucoDict,
        parameters=arucoParams)

    corners, ids, rejected = cv2.aruco.detectMarkers(frameReference, dict)
    cv2.aruco.drawDetectedMarkers(frameReference,corners)
    print(ids)
#    cv2.imshow("billede",frameReference)
    if corners: 
        #print(dist - arlo.read_front_ping_sensor())
        sleep(1)
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        tvec = tvec.reshape((3,))
        dist = np.linalg.norm(tvec)
        theta = np.arccos(np.dot((tvec/dist),ez))
        signfunc = np.sign(np.dot(tvec,ex))
        ang_deg = signfunc * np.rad2deg(theta)
        print(dist)

# Finished successfully



