# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import sys
import socket, os, os.path
import zmq

 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 90
rawCapture = PiRGBArray(camera, size=(320, 240))
area=0;

context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.bind("tcp://*:5555")


# allow the camera to warmup
time.sleep(0.5)
#message = socket.recv()
#main_start = time.time()

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	img = frame.array
        
	# show the frame
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)   #color space transformed to HSV

        lower_skin = np.array([100, 50, 0])  #lower bounds of skin pixels defined
        upper_skin = np.array([125, 255, 255]) #upper bound of skin pixels defined

        skinMask = cv2.inRange(hsv, lower_skin, upper_skin) #thresholds skin values by comparing with upper/lower bound.
        # res = cv2.bitwise_and(img, img, mask=mask)
        kernel = np.ones((5,5),np.uint8)    #creates a 5x5 kernel of 1's for morphological operations
        closed = cv2.morphologyEx(skinMask, cv2.MORPH_OPEN, kernel)  # Use morphological opening function with kernel on skin values
        closed = cv2.morphologyEx(closed, cv2.MORPH_CLOSE, kernel)	  #Use morpholoical closing after opening to remove noise
        contours, h  = cv2.findContours(closed, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #retrieve all contours and reconstructs a full heirarchy of nested contours.
        validContours = [];
        for cont in contours:
            if cv2.contourArea(cont) > 9000:  
                x,y,w,h = cv2.boundingRect(cont)   #gets coordinates of box around contours (fits a rectangle)
                #plot functions
                #print "area is %d" % (w*h)
                area = w*h                      #computes area of bounding box
                # if h/w > 0.75:
                validContours.append(cv2.convexHull(cont))       #corrects  convexivity defects (spaces between fingers)
                # rect = cv2.minAreaRect(cont)
                # box = cv2.cv.BoxPoints(rect)
                # validContours.append(np.int0(box))
        contours = validContours

        
        cv2.drawContours(img, contours, -1, (0, 255, 0), 2)  #displays the contours in the image (only used for debugging)
        cv2.imshow('capture', img)   #displays incoming video footage (only used for debugging)
        key = cv2.waitKey(1) & 0xFF    #waits for a keyboard event and continues program if key is pressed in the arguments time.
	
	

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	
        #  Send reply back to client
        socket.send(b"%d" % area)
