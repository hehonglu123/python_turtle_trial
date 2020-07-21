import time
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import threading
import numpy
import traceback
import cv2
import platform
import sys
import argparse

#Class that implements a single webcam
class Webcam_impl(object):
    #Init the camera being passed the camera number and the camera name
    def __init__(self):
        self._lock=threading.RLock()
        self.image=RRN.NewStructure("experimental.createwebcam2.WebcamImage")
        self._running1=False
        #Initialize the camera
        with self._lock:
            self._capture=cv2.VideoCapture(0)
            self._capture.set(3,320)
            self._capture.set(4,240)

    def start_streaming(self):
        self._running1=True
        self._streaming = threading.Thread(target=self.CaptureFrame)
        self._streaming.daemon = True
        self._streaming.start()


    def stop_streaming(self):
        self._running1 = False
        self._streaming.join()

    #Capture a frame and return a WebcamImage structure to the client
    def CaptureFrame(self):
        while self._running1:
            with self._lock:
                ret, frame=self._capture.read()
                if not ret:
                    raise Exception("Could not read from webcam")
                self.image.width=frame.shape[1]
                self.image.height=frame.shape[0]
                self.image.step=frame.shape[1]*3
                self.image.data=frame.reshape(frame.size, order='C')



def main():

    with RR.ServerNodeSetup("Webcam_Service",2355,argv=sys.argv) as node_setup:

        #Initialize the webcam host root object
        
        RRN.RegisterServiceTypeFromFile("experimental.createwebcam2")
        #create object
        c1=Webcam_impl()
        #register service with service name "Webcam", type "experimental.createwebcam2.Webcam", actual object: c1
        RRN.RegisterService("Webcam","experimental.createwebcam2.Webcam",c1)
        #start background streaming
        c1.start_streaming()

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("Server started, press enter to quit...")
        else:
            raw_input("Server started, press enter to quit...")

if __name__ == '__main__':
    main()
