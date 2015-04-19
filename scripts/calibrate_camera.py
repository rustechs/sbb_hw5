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
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
    # Get image size
    h,w,d = imgHSV.shape
    print 'Image Dimensions: (' + str(h) + ',' + str(w) + ')' 

    # Image center
    center = (int(h/2),int(w/2))

    # Threshold only the color we want
    # Green bowl
    # MIN = np.array([25,60,10])
    # MAX = np.array([85,255,255])

    # Blue block
    MIN = np.array([90,30,10])
    MAX = np.array([140,160,160])

    imgThresh = cv2.inRange(imgHSV, MIN, MAX)
    # output = cv2.bitwise_and(image, image, mask = mask)
        
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



    #<--- Harris Corner Detector --->
    # Create grayscale image used for Harris detection
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Compute Harris Corner Scores
    grayImage = np.float32(grayImage)
    dst = cv2.cornerHarris(grayImage,8,10,0.04)

    #Find Max 4 corners
    blockCorners = np.zeros((4, 2))
    for i in range(4)
    # Assumes dst is an 2D array
        maxHarris = dst.argmax(axis=0)

        # maxHarris = np.argmax(dst)
        # maxHarris = np.array([np.floor(np.divide(maxHarris,w)),np.remainder(maxHarris,w)])

        blockCorners[i,:] = maxHarris[0:2]
        dst[maxHarris[0],maxHarris[1]] = np.amin(dst)

    # cv2.imshow('dst',dst)

    # Find COM and Orientation
    blockCOM = np.mean(blockCorners, axis=0)

    leftNodes[blockCorners[:,0]<blockCOM[0]]                 #NODES TO THE LIEFT OF COM_x 
    orientation = leftNodes[0,:] - leftNodes[1,:]
    orientation = orientation / np.linalg.norm(orientation)  #Unit Vector


if __name__ == '__main__':

    if len(sys.argv) == 2:
        main(sys.argv[1])
    else:
        print('Please specify path to calibration image')
        sys.exit()
