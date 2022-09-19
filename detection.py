import cv2

print("OpenCV version = " + cv2.__version__)

dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Kamera ikke open")
    exit(-1)

while cv2.waitKey(5) == -1:
    retval, frame = cam.read() 

    if not retval:
        print("Intet billede")
        exit(-1)
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, dict)
    cv2.aruco.drawDetectedMarkers(frame,corners)
    print(ids)
    cv2.imshow("billede",frame)

