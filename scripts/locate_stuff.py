#!/usr/bin/env python

import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sbb_hw5.srv import *
import string

# import tesseract

class locate_stuff():

    def __init__(self):
        # Listen to camera image topic
        self.img_sub = rospy.Subscriber('cameras/right_hand_camera/image',Image,self.parseFrame)
        # self.img_sub = rospy.Subscriber('cameras/left_hand_camera/image',Image,self.parseFrame)
        # Provide a service to return newest bowl/block pose
        self.img_serv = rospy.Service('find_stuff',FindStuffSrv,self.servCall)
        # Publish overlayed image
        # self.img_pub = rospy.Publisher('/cv/bowl_block',Image,queue_size=10)
        self.img_pub = rospy.Publisher('/robot/xdisplay',Image,queue_size=10)
        # Create a image conversion bridge
        self.br = CvBridge()
        self.blockLoc = (False,0,0,0)
        self.bowlLoc = (False,0,0,0)
        self.center = (0,0)
        # self.ocrAPI = tesseract.TessBaseAPI()
        # self.ocrAPI.Init("/usr/share/tesseract-ocr/","eng",tesseract.OEM_DEFAULT)
        # self.ocrAPI.SetPageSegMode(tesseract.PSM_SINGLE_CHAR)
        # self.ocrAPI.SetVariable("tessedit_pageseg_mode", "7")
       	# self.ocrAPI.SetVariable("tessedit_char_whitelist", "0123456789")
        self.lastBlock = None

    # Camera frame topic callback
    # Find bowl and block on each frame refresh
    def parseFrame(self,data):
        # Convert image into openCV format
        try:
            self.img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
            raise
        
        # Get frame center
        self.center = (int(self.img.shape[1]/2),int(self.img.shape[0]/2))

        # Convert image to HSV
        self.imgHSV = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)

        # Detect block and bowl, save to state variables
        self.blockLoc = self.findBlock()
        self.bowlLoc = self.findBowl()
       
        #rospy.loginfo("Bowl Location: %s" % str(self.bowlLoc))
        #rospy.loginfo("Block Location: %s" % str(self.blockLoc))

        # Publish a pretty picture of what we found
        self.pubOverlayImg()

    def pubOverlayImg(self):
   
        # Show unmasked img
        # imgShow = img

        # Save image size
        hT,wT,cT = self.img.shape
        
        # Set up font for text
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL

        # Create masked image for bowl
        imgShowBowl = cv2.bitwise_and(self.img,self.img,mask = self.imgThreshBowl)

        # Add block to masked image
        # Generate box threshold
    	imgShowBlock = np.zeros((hT,wT,3), np.uint8)

    	if self.blockLoc[0]:
    		cv2.drawContours(imgShowBlock,[self.box],0,(255,255,255),-1)

    	imgShowBlock = cv2.cvtColor(imgShowBlock,cv2.COLOR_BGR2GRAY)
    	retval,imgShowBlock = cv2.threshold(imgShowBlock,128,255,cv2.THRESH_BINARY)
        imgShowBlock = cv2.bitwise_and(self.img,self.img,mask = imgShowBlock)
        imgShow = cv2.bitwise_or(imgShowBowl,imgShowBlock)

        # Plot the center
        cv2.circle(imgShow,self.center,5,(30,30,240),-1)
        
        # Plot a center line
        cv2.line(imgShow,(self.center[0],0),(self.center[0],self.img.shape[0]),(30,30,240),1)

        # If block was found, show it
        if self.blockLoc[0]:
            bX = self.blockLoc[1]+self.center[0]
            bY = self.center[1]-self.blockLoc[2]
            bT = self.blockLoc[3]
            bW = self.rectW
            cv2.circle(imgShow,(bX,bY),5,(240,50,30),-1)            
            cv2.drawContours(imgShow,[self.box],0,(0,165,200),2)
            #cv2.line(imgShow,tuple(np.int_((bX-bW*np.cos(bT),bY-bW*np.sin(bT)))),tuple(np.int_((bX+bW*np.cos(bT),bY+bW*np.sin(bT)))),(0,165,200),1)
            #cv2.line(imgShow,tuple(np.int_((bX-bW*np.cos(bT+np.pi/2),bY-bW*n(0,165,200),1p.sin(bT+np.pi/2)))),tuple(np.int_((bX+bW*np.cos(bT+np.pi/2),bY+bW*np.sin(bT+np.pi/2)))),(0,165,200),1)
            
            pt1 = [bX,bY]
            pt2 = pt1 + (self.box[0] - self.box[1])
            # print 'pt1 is looks like ... '  + str(pt1)
            # print 'pt2 is looks like ... '  + str(pt2)
            cv2.line(imgShow,tuple(np.int_(pt1)),tuple(np.int_(pt2)),(0,165,200),1)
            # cv2.circle(imgShow,tuple(self.box[0]),4,(240,50,30),-1)
            # cv2.circle(imgShow,tuple(self.box[2]),4,(240,50,30),-1)
            # cv2.circle(imgShow,tuple(self.box[1]),4,(30,50,240),-1)
            # cv2.circle(imgShow,tuple(self.box[3]),4,(30,50,240),-1)
            blockStr = 'Block: ' + str(self.blockLoc)
            cv2.putText(imgShow,blockStr,(5,60),font,1,(255,255,255),1,cv2.CV_AA)
        # If bowl was found, show it
        if self.bowlLoc[0]:
            cv2.circle(imgShow,(self.bowlLoc[1]+self.center[0],self.center[1]-self.bowlLoc[2]),5,(30,240,30),-1)
            bowlStr = 'Bowl: ' + str(self.bowlLoc)
            cv2.putText(imgShow,bowlStr,(5,30),font,1,(255,255,255),1,cv2.CV_AA)
        
        # Publish image
        self.img_pub.publish(self.br.cv2_to_imgmsg(cv2.resize(imgShow,(1024,600)), "bgr8"))
            
    # Deal with service call by parsing argument, returning latest location for item
    def servCall(self,data):
        # Use latest image to look for stuff, return it
        if data.item == 'bowl':
            return FindStuffSrvResponse(*self.bowlLoc)
        elif data.item == 'block':
            return FindStuffSrvResponse(*self.blockLoc)
        else:
            raspy.logerr("Incorrect service call argument, use either bowl or block")
            raise

    # Find bowl using basic color thresholding centroid method
    # If found, orientation defaults to 0
    # Returns type (found,x,y,t)
    def findBowl(self):
        # Green Color
        MIN = np.array([25,60,10])
        MAX = np.array([85,180,115])

        self.imgThreshBowl = cv2.inRange(self.imgHSV,MIN,MAX)

        # Check if there's enough green pixels to constitute a bowl
        if float(cv2.countNonZero(self.imgThreshBowl))/(self.img.shape[0]*self.img.shape[1]) >= 0.05:
            m = cv2.moments(self.imgThreshBowl)
            x = int(m['m10']/m['m00'])
            y = int(m['m01']/m['m00'])
            dx = x - self.center[0]
            dy = self.center[1] - y
            return (True,dx,dy,0)
        else:
            return (False,0,0,0)

    # Find block using corner detection (with color thresholding)
    # Returns tuple (found,x,y,t)
    def findBlock(self):
        # Blue Color 
        MIN = np.array([90,30,15])
        MAX = np.array([140,115,130])

        # Color threshold
        self.imgThreshBlock = cv2.inRange(self.imgHSV,MIN,MAX)

        # Set up ROI
        ROI = 0.75
        leftW = int(self.img.shape[1]*(1-ROI)/2)
        rightW = int(self.img.shape[1]*(0.5+ROI/2))
        topW = int(self.img.shape[0]*(1-ROI)/2)
        botW = int(self.img.shape[0]*(0.5+ROI/2))
        self.imgThreshBlock[:,:leftW] = 0
        self.imgThreshBlock[:,rightW:] = 0
        self.imgThreshBlock[:topW,:] = 0
        self.imgThreshBlock[botW:,:] = 0

        # Check if there's enough green pixels to constitute a block
        if float(cv2.countNonZero(self.imgThreshBlock))/(self.img.shape[0]*self.img.shape[1]) >= 0.015:
            # m = cv2.moments(self.imgThreshBlock)
            # print "I see a block"
            # dx = x - self.center[0]
            # dy = self.center[1] - y
            
            # Find the right contour
            c,h = cv2.findContours(self.imgThreshBlock,1,2)
            
            # print c
            
            maxcArea = 0
            for i in range(len(c)):
                 cAreas = cv2.contourArea(c[i])
                 if maxcArea<cAreas:
                    maxcArea = cAreas
                    indexcArea = i
              
            cnt = c[indexcArea]

            rect = cv2.minAreaRect(cnt)
           
            self.box = cv2.cv.BoxPoints(rect)
            self.box = np.int0(self.box)

            # Do OCR
            # self.OCR()
            
            # print(self.box)

            x,y = rect[0]
           
            self.rectW,self.rectH = rect[1]

            maxSide = 100
            if (self.rectW <= maxSide and self.rectH <= maxSide and abs(self.rectW-self.rectH) <= 10):

                x = int(x)
                y = int(y)

                pt21 = self.box[0]-self.box[1]
                t = (np.arctan2(pt21[1],pt21[0])) % (np.pi/2)

                dx = x - self.center[0]
                dy = self.center[1] - y
                return (True,dx,dy,t)

            else:
                return (False,0,0,0)
        else:
            return (False,0,0,0)


    # Recognise numbers on block
    # def OCR(self):
    	# import pdb; pdb.set_trace()

    	# Convert to openCV 1 image type
    	# hT,wT,cT = self.img.shape

    	# Generate box threshold
    	# imgBlockMask = np.zeros((hT,wT,3), np.uint8)

    	# cv2.drawContours(imgBlockMask,[self.box],0,(255,255,255),-1)

    	# imgBlockMask = cv2.cvtColor(imgBlockMask,cv2.COLOR_BGR2GRAY)
    	# r,imgBlockMask = cv2.threshold(imgBlockMask,128,255,cv2.THRESH_BINARY)

    	# imgOCRBlock = cv2.bitwise_and(self.img,self.img,mask = imgBlockMask)

    	# imgOCRBlock = cv2.cvtColor(imgOCRBlock,cv2.COLOR_BGR2GRAY)
    	# imgOCRNumMask = cv2.adaptiveThreshold(imgOCRBlock,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,3,5)
    	# r,imgOCRNumMask = cv2.threshold(imgOCRBlock,50,255,cv2.THRESH_BINARY)

    	# imgOCR = cv2.bitwise_and(imgOCRBlock,imgOCRBlock,mask = imgOCRNumMask)

    	# Show the OCR threshold image
    	# cv2.imshow('Threshold Image',imgOCR)
    	# cv2.waitKey(5)

    	# ocrImg = cv.CreateImageHeader((wT,hT), cv.IPL_DEPTH_8U, cT)
    	# cv.SetData(ocrImg,imgOCR.tostring(),imgOCR.dtype.itemsize * cT * (wT))
    	# tesseract.SetCvImage(ocrImg,self.ocrAPI)

    	# Try to find some numbers
     	# self.lastBlock=self.ocrAPI.GetUTF8Text()
     	# self.lastBlock.strip(string.whitespace).replace("\n", "")
     	# print 'Block number is: ' + self.lastBlock
        
 
# Main loop
def main():
    
    # Init node
    rospy.init_node('locate_stuff', anonymous = True)
    
    # Instantiate a node
    locator = locate_stuff()
    
    print "Ready to find stuff!"

    # Wait for either a topic update or a service call
    rospy.spin()


if __name__ == '__main__':
    main()
