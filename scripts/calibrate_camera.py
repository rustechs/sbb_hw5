#!/usr/bin/env python

import cv2
import sys
import numpy as np

def main(img_path):

    try:

        # Try to open image as BGR (default behaviour)
        img = cv2.imread(img_path)
        
        # Check if image was opened
        
        if img == None:
            raise
    except:
        print('Incorrect image path!')  
        sys.exit() # Show the image
        
    cv2.imshow('Calibration Image',img)
    cv2.waitKey()
    cv2.destroyAllWindows()
    
    # Convert to HSV
    imgHVT = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
    # Get image size
    h,w,d = imgHVT.shape
    print 'Image Dimensions: (' + str(h) + ',' + str(w) + ')' 

    # Image center
    center = (int(h/2),int(w/2))

    # Threshold only the color we want
    MIN = np.array([25,60,10])
    MAX = np.array([85,255,255])

    imgThresh = cv2.inRange(imgHVT, MIN, MAX)
        
    cv2.imshow('Threshold Image',imgThresh)
    cv2.waitKey()
    cv2.destroyAllWindows()
     
    # Find centroid with zeroth order moment
    m = cv2.moments(imgThresh)
    centroid = (int(m['m10']/m['m00']), int(m['m01']/m['m00'])) 
    print 'Object centroid at: ' + str(centroid)

    # Show image with centroid overlay
    img
    cv2.circle(img,centroid,5,(120,255,255),-1)  
    cv2.imshow('Threshold Image w/ Centroid',img)
    cv2.waitKey()
    cv2.destroyAllWindows()

    # Print the x,y delta distances
    delta = tuple(np.subtract(centroid,center))

    print 'Object centroid delta from center: ' + str(delta)

if __name__ == '__main__':

    if len(sys.argv) == 2:
        main(sys.argv[1])
    else:
        print('Please specify path to calibration image')
        sys.exit()
