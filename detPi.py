import cv2

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


print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

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

