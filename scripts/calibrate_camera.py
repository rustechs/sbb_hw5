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
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
    # Get image size
    h,w,d = img.shape
    print 'Image Dimensions: (' + str(h) + ',' + str(w) + ')' 

    # Image center
    center = (h/2,w/2)

    # Threshold only the color we want
    MIN = np.array([95,50,50])
    MAX = np.array([130,255,255])

    imgThresh = cv2.inRange(img, MIN, MAX)
        
    cv2.imshow('Threshold Image',imgThresh)
    cv2.waitKey()
    cv2.destroyAllWindows()
     
    # Find centroid with zeroth order moment
    m = cv2.moments(imgThresh)
    centroid = (m['m10']/m['m00'], m['m01']/m['m00']) 
    print 'Object centroid at: ' + str(centroid)

    # Print the x,y delta distances
    delta = tuple(np.subtract(centroid,center))

    print 'Object centroid delta from center: ' + str(delta)

if __name__ == '__main__':

    if len(sys.argv) == 2:
        main(sys.argv[1])
    else:
        print('Please specify path to calibration image')
        sys.exit()
