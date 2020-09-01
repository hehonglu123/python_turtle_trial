from RobotRaconteur.Client import *     #import RR client library
import cv2
import sys

#Function to take the data structure returned from the Webcam service
#and convert it to an OpenCV array
def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2

#Main program

url='rr+tcp://localhost:2355/?service=Webcam'
#take url from command line
if (len(sys.argv)>=2):
    url=sys.argv[1]

#connect to service given url, returned with an RR object, defined with service definition    

cam_sub=RRN.SubscribeService(url)
cam_obj=cam_sub.GetDefaultClientWait(5)




while True:
    if (not cam_obj.image is None):
        cv2.imshow("Image",WebcamImageToMat(cam_obj.image))
    
    if cv2.waitKey(50)!=-1:
        break
cv2.destroyAllWindows()


