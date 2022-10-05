# -*- coding: utf-8 -*-
"""
Created on Wed Oct  5 17:28:17 2022

@author: flemm
"""

import cv2
from threading import Thread


#Pipline
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